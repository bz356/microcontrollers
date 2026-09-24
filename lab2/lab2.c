/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA graphics library
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
#include "hardware/spi.h"
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
#define hitBottom(b) (b>int2fix15(480))
#define hitTop(b) (b<int2fix15(0))
#define hitLeft(a) (a<int2fix15(0))
#define hitRight(a) (a>int2fix15(640))

// the color of the boid
char color = WHITE ;

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size] ;

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size] ;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

// SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// DMA channel for sound
int data_chan ;

// Boid on core 0
fix15 boid0_x ;
fix15 boid0_y ;
fix15 boid0_vx ;
fix15 boid0_vy ;

#define BALL_RADIUS 4
#define PEG_RADIUS 6

fix15 peg_x = int2fix15(320) ;
fix15 peg_y = int2fix15(240) ;

fix15 gravity = float2fix15(0.37) ;
fix15 bounciness = float2fix15(0.5) ;

// Boid on core 1
fix15 boid1_x ;
fix15 boid1_y ;
fix15 boid1_vx ;
fix15 boid1_vy ;

// Create a semaphore
semaphore_t draw_semaphore ;

// Play sound with DMA
void playSound() {
  dma_channel_abort(data_chan) ;
  dma_channel_set_read_addr(data_chan, DAC_data, false) ;
  dma_channel_set_trans_count(data_chan, sine_table_size, true) ;
}

// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  (void)direction ;

  // Start in top center of screen instead of center
  *x = int2fix15(320) ;
  *y = int2fix15(20) ;

  // Randomized horizontal velocity
  if (rand() & 1) {
    *vx = float2fix15(0.2) ; 
  } 
  else {
    *vx = float2fix15(-0.2) ;
  }

  // Ball is dropped with zero y-velocity
  *vy = 0 ;
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

  // Check for collision with peg
  fix15 dx = *x - peg_x ;
  fix15 dy = *y - peg_y ;

  fix15 collision_distance = int2fix15(BALL_RADIUS + PEG_RADIUS) ;

  if ((absfix15(dx) < collision_distance) &&
      (absfix15(dy) < collision_distance)) {

    float dx_float = fix2float15(dx) ;
    float dy_float = fix2float15(dy) ;

    float distance = sqrt((dx_float * dx_float) + (dy_float * dy_float)) ;

    if ((distance < (BALL_RADIUS + PEG_RADIUS)) && (distance > 0)) {

      fix15 normal_x = float2fix15(dx_float / distance) ;
      fix15 normal_y = float2fix15(dy_float / distance) ;

      fix15 intermediate_term =
          -2 * (multfix15(normal_x, *vx) + multfix15(normal_y, *vy)) ;

      // Move ball just outside the peg
      *x = peg_x + multfix15(normal_x, int2fix15(PEG_RADIUS + BALL_RADIUS + 1)) ;
      *y = peg_y + multfix15(normal_y, int2fix15(PEG_RADIUS + BALL_RADIUS + 1)) ;

      // Change velocity so the ball bounces
      *vx = *vx + multfix15(normal_x, intermediate_term) ;
      *vy = *vy + multfix15(normal_y, intermediate_term) ;

      // Lose some energy during the bounce
      *vx = multfix15(bounciness, *vx) ;
      *vy = multfix15(bounciness, *vy) ;

      // Play sound when ball hits peg
      playSound() ;
    }
  }

  // If ball falls off bottom of screen, drop again from top
  if (*y > int2fix15(480)) {
    spawnBoid(x, y, vx, vy, 0) ;
    return ;
  }

  // Make gravity increase downward velocity every frame
  *vy = *vy + gravity ;
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
      // PT_SEM_SDK_SIGNAL(pt, &draw_semaphore) ;

      // update boid's position and velocity
      wallsAndEdges(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy) ;

      // draw the peg
      fillCircle(fix2int15(peg_x), fix2int15(peg_y), PEG_RADIUS, WHITE); 

      // draw the ball
      fillCircle(fix2int15(boid0_x), fix2int15(boid0_y), BALL_RADIUS, BLUE); 

      // draw the boundaries
      // drawArena() ;

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

  // initialize stdio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  // taken all from dma-demo ---------------------------
  // Initialize SPI channel (channel, baud rate set to 20MHz)
  spi_init(SPI_PORT, 20000000) ;

  // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  // Map SPI signals to GPIO ports
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);


  // Build sine table and DAC data table
  int i ;
  for (i=0; i<sine_table_size; i++){
    raw_sin[i] = (int)(2047 * sin((float)i*6.283/(float)sine_table_size) + 2047);
    DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff) ;
  }

 
  // Select DMA channel
  data_chan = dma_claim_unused_channel(true);

  // Setup the data channel
  dma_channel_config c2 = dma_channel_get_default_config(data_chan);
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);
  channel_config_set_read_increment(&c2, true);
  channel_config_set_write_increment(&c2, false);

  // Configure DMA timer
  dma_timer_set_fraction(0, 0x0017, 0xffff) ;

  // 0x3b means timer0
  channel_config_set_dreq(&c2, 0x3b);

  dma_channel_configure(
      data_chan,
      &c2,
      &spi_get_hw(SPI_PORT)->dr,
      DAC_data,
      sine_table_size,
      false
  );
  // ------------------------------------
  // Initialize the semaphore
  // Arguments: pointer to sem, initial count, max count
  sem_init(&draw_semaphore, 0, 1) ;

  // start core 1 
  // multicore_reset_core1();
  // multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
}