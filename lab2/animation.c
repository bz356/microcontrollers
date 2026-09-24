
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS (WEEK 1 FOCUS)

 VGA (resistors for voltage division to VGA analog input)
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - GND     ---> VGA GND

  DAC
  - GPIO 5  ---> CS, DAC pin 2
  - GPIO 6  ---> SCLK, DAC pin 3
  - GPIO 7  ---> MOSI, DAC pin 4
  - GPIO 4  ---> MISO, NC
  - +3.3V   ---> DAC VDD
  - GND     ---> DAC GND and LDAC

  Rotary Encoder (turn on internal pull-ups for A, B, and SWITCH)
  - GPIO 10 ---> A
  - GPIO 11 ---> B
  - GPIO 12 ---> SWITCH
  - GND     ---> C, other SWITCH PIN


 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "VGA/vga16_graphics_v3.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
#include "pico/sync.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// Rotary Encoder
#define ENC_A  10
#define ENC_B  11
#define ENC_SW 12
#define ENC_SW_DEBOUNCE_US 20000 // 20 ms

volatile int enc_count = 0; // number shown on VGA
static volatile uint8_t enc_state; // state of pins A and B written in last 2 bits as AB
static volatile int8_t enc_accum; // quarter step count of pins A and B

volatile bool enc_sw_pressed = false; // flag set by interrupt when switch is pressed
static volatile uint32_t enc_sw_last_edge;

// read from old to new, +1 or -1 on valid one-step turns and 0 for no turn or impossible two-step turn

static const int8_t enc_table[16] = {
  // new: 00, 01, 10 11
          0, -1,  1,  0, // old 00
          1,  0,  0, -1, // old 01
         -1,  0,  0,  1, // old 10
          0,  1, -1,  0, // old 11
};

void enc_callback(uint gpio, uint32_t events)
{
  if (gpio == ENC_A || gpio == ENC_B) {
    uint8_t new_state = (gpio_get(ENC_A) << 1) | gpio_get(ENC_B);
    enc_accum += enc_table[(enc_state << 2) | new_state]; // increments, decrements, or keeps enc_accum the same depending on lookup table
    enc_state = new_state;

    if (new_state == 0b11) 
    {
      if (enc_accum >= 4) 
      {
        enc_count++;
      }
      else if (enc_accum <= -4)
      {
        enc_count--;
      }
      enc_accum = 0;
    }
  }

  else if (gpio == ENC_SW) {
    uint32_t now = time_us_32();

    if ((events & GPIO_IRQ_EDGE_FALL) && (now - enc_sw_last_edge > ENC_SW_DEBOUNCE_US)) 
    {
      enc_sw_pressed = true;
    }

    enc_sw_last_edge = now;
  }
}

void enc_init(void)
{
  gpio_init(ENC_A) ;  gpio_set_dir(ENC_A, GPIO_IN) ;  gpio_pull_up(ENC_A) ;
  gpio_init(ENC_B) ;  gpio_set_dir(ENC_B, GPIO_IN) ;  gpio_pull_up(ENC_B) ;
  gpio_init(ENC_SW) ; gpio_set_dir(ENC_SW, GPIO_IN) ; gpio_pull_up(ENC_SW) ;

  sleep_ms(1) ;   // let the pull-ups settle before reading the start state
  enc_state = (gpio_get(ENC_A) << 1) | gpio_get(ENC_B) ;
  enc_accum = 0 ;

  // the first call registers the callback; the others just enable their pins
  gpio_set_irq_enabled_with_callback(ENC_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &enc_callback) ;
  gpio_set_irq_enabled(ENC_B,  GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true) ;
  gpio_set_irq_enabled(ENC_SW, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true) ;
}

// the color of the boid
char color = WHITE ;

// Boid on core 0
fix15 boid0_x ;
fix15 boid0_y ;
fix15 boid0_vx ;
fix15 boid0_vy ;

// Boid on core 1
fix15 boid1_x ;
fix15 boid1_y ;
fix15 boid1_vx ;
fix15 boid1_vy ;

// Create a semaphore
semaphore_t draw_semaphore ;

// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(240) ;
  // Choose left or right
  if (direction) *vx = int2fix15(3) ;
  else *vx = int2fix15(-3) ;
  // Moving down
  *vy = int2fix15(1) ;
}

// Draw the boundaries
void drawArena() {
  drawVLine(100, 100, 280, WHITE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Reverse direction if we've hit a wall
  if (hitTop(*y)) {
    *vy = (-*vy) ;
    *y  = (*y + int2fix15(5)) ;
  }
  if (hitBottom(*y)) {
    *vy = (-*vy) ;
    *y  = (*y - int2fix15(5)) ;
  } 
  if (hitRight(*x)) {
    *vx = (-*vx) ;
    *x  = (*x - int2fix15(5)) ;
  }
  if (hitLeft(*x)) {
    *vx = (-*vx) ;
    *x  = (*x + int2fix15(5)) ;
  } 

  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.4\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // update boid color
        if ((user_input > 0) && (user_input < 16)) {
          color = (char)user_input ;
        }
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Spawn a boid
    spawnBoid(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy, 0);

    while(1) {
      // Wait for the signal that the buffer's changed
      PT_YIELD_UNTIL(pt, draw_start_signal()) ;
      // Clear the buffer
      clearLowFrame(0, BLACK);
      // Signal core 1 that it can start drawing
      PT_SEM_SDK_SIGNAL(pt, &draw_semaphore) ;
      // update boid's position and velocity
      wallsAndEdges(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy) ;
      // draw the boid at its new position
      fillCircle(fix2int15(boid0_x), fix2int15(boid0_y), 15, color); 
      // draw the boundaries
      drawArena() ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Spawn a boid
    spawnBoid(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy, 1);

    while(1) {
      // Wait for the signal from core 0
      PT_SEM_SDK_WAIT(pt, &draw_semaphore) ;
      // update boid's position and velocity
      wallsAndEdges(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy) ;
      // draw the boid at its new position
      fillCircle(fix2int15(boid1_x), fix2int15(boid1_y), 15, color); 
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  set_sys_clock_khz(150000, true) ;
  // initialize stio
  stdio_init_all() ;

  // initialize rotary encoder
  enc_init();

  // initialize VGA
  initVGA() ;

  // Initialize the semaphore
  // Arguments: pointer to sem, initial count, max count
  sem_init(&draw_semaphore, 0, 1) ;

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
