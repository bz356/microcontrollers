
/*
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
#include "hardware/spi.h"

// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0f)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0f)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Rotary Encoder
#define ENC_A  10
#define ENC_B  11
#define ENC_SW 12
#define ENC_SW_DEBOUNCE_US 20000 // 20 ms

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size] ;

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size] ;

// Pointer to the address of the DAC data table
unsigned short * address_pointer = &DAC_data[0] ;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Ball definition
#define MAX_BALLS 10000
// Ball count at reset: tune to the largest value that keeps the LED off
#define START_BALLS 9700

typedef struct /*__attribute__((packed))*/ Ball {
  fix15 x; 
  fix15 y;
  fix15 vx; 
  fix15 vy; 

  int16_t last_peg; // to keep track of when to "thunk"
} Ball;

Ball balls[MAX_BALLS];
int current_ball_count = START_BALLS;

#define NUM_ROWS 16
#define PEG_START_Y 60 // where the first peg starts
#define ROW_SPACE 19 // space between rows (vertical distance)
#define PEG_SPACE 38 // space between pegs 

#define NUM_PEGS 136 // 1 + 2 + 3... + 16 = 136

typedef struct Peg {
  fix15 x;
  fix15 y;
} Peg;

Peg pegs[NUM_PEGS];


volatile int enc_delta = 0; // clicks since the animation thread last checked (+ = clockwise)
static volatile uint8_t enc_state; // state of pins A and B written in last 2 bits as AB
static volatile int8_t enc_accum; // quarter step count of pins A and B

volatile bool enc_sw_pressed = false; // flag set by interrupt when switch is pressed
static volatile uint32_t enc_sw_last_edge; // time of last switch edge for debouncing

// read from old to new, +1 or -1 on valid one-step turns and 0 for no turn or impossible two-step turn

static const int8_t enc_table[16] = {
  // new: 00, 01, 10 11
          0, -1,  1,  0, // old 00
          1,  0,  0, -1, // old 01
         -1,  0,  0,  1, // old 10
          0,  1, -1,  0, // old 11
};

// Histogram definitions and variables

#define NUM_BUCKETS (NUM_ROWS + 1)

#define LAST_PEG_Y (PEG_START_Y + (NUM_ROWS - 1) * ROW_SPACE)
#define LAST_ROW_FIRST_X (320 - ((NUM_ROWS - 1) * PEG_SPACE) / 2)

#define HIST_TOP (LAST_PEG_Y + PEG_RADIUS + BALL_RADIUS + 6)
#define HIST_BOTTOM 475
#define HIST_HEIGHT (HIST_BOTTOM - HIST_TOP)
#define BAR_WIDTH (PEG_SPACE - 8)

// int histogram[NUM_BUCKETS] = {0};
// uint32_t ball_count = 0;
// int histogram_max = 0;

int histogram[2][NUM_BUCKETS] = {0};  // one row per core
uint32_t ball_count[2] = {0};

// button cycles through different modes after clicking
typedef enum { MODE_BALLS, MODE_BOUNCE, MODE_GRAVITY, NUM_MODES } Mode;
Mode mode = MODE_BALLS;
const char *mode_names[NUM_MODES] = { "Ball count", "Bounciness", "Gravity" };

// Frame timing
#define FRAME_US 16667 // 1/60 s in microseconds
#define LED_PIN PICO_DEFAULT_LED_PIN

void enc_callback(uint gpio, uint32_t events)
{
  if (gpio == ENC_A || gpio == ENC_B) {
    uint8_t new_state = (gpio_get(ENC_A) << 1) | gpio_get(ENC_B);
    enc_accum += enc_table[(enc_state << 2) | new_state]; // increments, decrements, or keeps enc_accum the same depending on lookup table
    enc_state = new_state;

    if (new_state == 0b11) 
    {
      if (enc_accum >= 4) {
        enc_delta++;
      }

      else if (enc_accum <= -4) {
        enc_delta--;
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
// char color = WHITE ;

// Boid on core 0
// fix15 boid0_x ;
// fix15 boid0_y ;
// fix15 boid0_vx ;
// fix15 boid0_vy ;

// // Boid on core 1
// fix15 boid1_x ;
// fix15 boid1_y ;
// fix15 boid1_vx ;
// fix15 boid1_vy ;

#define BALL_RADIUS 2
#define PEG_RADIUS 6

#define COLLISION_DISTANCE int2fix15(BALL_RADIUS + PEG_RADIUS)
#define COLLISION_SQUARED  multfix15(COLLISION_DISTANCE, COLLISION_DISTANCE)

// fix15 peg_x = int2fix15(320) ;
// fix15 peg_y = int2fix15(240) ;

fix15 gravity = float2fix15(0.37) ;
fix15 bounciness = float2fix15(0.5) ;

int global_ctr_chan;
int global_data_chan;

// Create a semaphore
semaphore_t draw_semaphore ;
semaphore_t done_semaphore ;

static void dmaSetup(void) {
  // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Build sine table and DAC data table
    int i ;
    for (i=0; i<(sine_table_size); i++){
        raw_sin[i] = (int)(2047 * sin((float)i*6.283/(float)sine_table_size) + 2047); //12 bit
        DAC_data[i] = DAC_config_chan_B | (raw_sin[i] & 0x0fff) ;
    }

    // Select DMA channels
    int data_chan = dma_claim_unused_channel(true);;
    int ctrl_chan = dma_claim_unused_channel(true);;
    global_ctr_chan = ctrl_chan;
    global_data_chan = data_chan;

    // Setup the control channel
    dma_channel_config c = dma_channel_get_default_config(ctrl_chan);   // default configs
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);             // 32-bit txfers
    channel_config_set_read_increment(&c, false);                       // no read incrementing
    channel_config_set_write_increment(&c, false);                      // no write incrementing
    channel_config_set_chain_to(&c, data_chan);                         // chain to data channel

    dma_channel_configure(
        ctrl_chan,                          // Channel to be configured
        &c,                                 // The configuration we just created
        &dma_hw->ch[data_chan].read_addr,   // Write address (data channel read address)
        &address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers
        false                               // Don't start immediately
    );

    // Setup the data channel
    dma_channel_config c2 = dma_channel_get_default_config(data_chan);  // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);            // 16-bit txfers
    channel_config_set_read_increment(&c2, true);                       // yes read incrementing
    channel_config_set_write_increment(&c2, false);                     // no write incrementing
    // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
    // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
    dma_timer_set_fraction(0, 0x0017, 0xffff) ;
    // 0x3b means timer0 (see SDK manual)
    channel_config_set_dreq(&c2, 0x3b);                                 // DREQ paced by timer 0
    // chain to the controller DMA channel
    // channel_config_set_chain_to(&c2, ctrl_chan);                        // Chain to control channel


    dma_channel_configure(
        data_chan,                  // Channel to be configured
        &c2,                        // The configuration we just created
        &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
        DAC_data,                   // The initial read address
        sine_table_size,            // Number of transfers
        false                       // Don't start immediately.
    );


  // // start the control channel SOUND ON
  dma_start_channel_mask(1u << ctrl_chan) ;
  
}

static void thunk(void) {
  if(!dma_channel_is_busy(global_data_chan)){
     dma_start_channel_mask(1u << global_ctr_chan) ;
  }
}

// ==================================================
// Histogram
// ==================================================


// Bucket 0 is left of the first peg w last bucket  right of the rightmost peg 
// All others are between pairs of pegs.
static int bucketForX(int x) {
    if (x < LAST_ROW_FIRST_X) {
        return 0;
    }

    int bucket = 1 + (x - LAST_ROW_FIRST_X) / PEG_SPACE;

    if (bucket >= NUM_BUCKETS) {
        return NUM_BUCKETS - 1;
    }

    return bucket;
}

void addToHistogram(int x) {
    histogram[get_core_num()][bucketForX(x)]++;
}

void drawHistogram(void) {
    int totals[NUM_BUCKETS];
    int max = 0;
    for (int i = 0; i < NUM_BUCKETS; i++) {
        totals[i] = histogram[0][i] + histogram[1][i];
        if (totals[i] > max) max = totals[i];
    }
    if (max > 0) {
        for (int i = 0; i < NUM_BUCKETS; i++) {
            int bar_height = (totals[i] * HIST_HEIGHT) / max;
            if (bar_height > 0) {
                int center_x = LAST_ROW_FIRST_X + i * PEG_SPACE - PEG_SPACE / 2;
                drawRect(center_x - BAR_WIDTH / 2, HIST_BOTTOM - bar_height,
                         BAR_WIDTH, bar_height, BLUE);
            }
        }
    }
    drawHLine(0, HIST_BOTTOM, 640, WHITE);
}

// Clear the histogram and the total-fallen count
void resetStats(void) {
  memset(histogram, 0, sizeof(histogram));   // clears both cores' rows
  ball_count[0] = 0;
  ball_count[1] = 0;
}


// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  (void)direction ;

  // Start in top center of screen instead of center
  *x = int2fix15(320) ;
  *y = int2fix15(30) ;

  // Randomized horizontal velocity
  // float random_vx = -0.2f + ((float)rand() / 100) * 0.6f;
  int32_t offset_milli = (int32_t)(rand() % 401) - 200;
  if (offset_milli == 0) {
    offset_milli += (rand() & 1) ? 1 : -1;   // 50/50: either 1 or -1;
  }
  //float random_vx = -0.2f + ((float)offset_milli / 1000.0f) * 0.6f;
  *vx = float2fix15(offset_milli / 1000.0f) ;

  // Ball is dropped with zero y-velocity
  *vy = 0 ;
}

void spawnBall(Ball *ball) {
  ball->x = int2fix15(320);
  ball->y = int2fix15(0);

  // Randomized horizontal velocity
  // int32_t offset_milli = (int32_t)(rand() % 401) - 200;
  // if (offset_milli == 0) {
  //   offset_milli += (rand() & 1) ? 1 : -1;   // 50/50: either 1 or -1;
  // }

  // dont use mod, it's too expensive! 
  ball->vx = (rand() & 0xffff) - int2fix15(1);

  // ball->vx = float2fix15(offset_milli / 1000.0f);
  ball->vy = 0; 
  ball->last_peg = -1; // at the top, did not hit any peg yet 
}

// Keep a fix15 value between lo and hi
fix15 clampFix(fix15 v, fix15 lo, fix15 hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}



// Add or remove balls; new balls start fresh at the top
void changeBallCount(int clicks) {
  int n = current_ball_count + clicks;
  if (n < 0) n = 0;
  if (n > MAX_BALLS) n = MAX_BALLS;
  for (int i = current_ball_count; i < n; i++) {
    spawnBall(&balls[i]);
  }
  current_ball_count = n;
}

// Called once per frame: apply button presses and encoder clicks
void handleInput(void) {
  // Button: go to the next mode
  if (enc_sw_pressed) {
    enc_sw_pressed = false;
    mode = (mode + 1) % NUM_MODES;
  }

  // Grab the clicks with interrupts off so the ISR can't change enc_delta mid-read
  uint32_t irq_state = save_and_disable_interrupts();
  int clicks = enc_delta;
  enc_delta = 0;
  restore_interrupts(irq_state);

  if (clicks == 0) return;

  switch (mode) {
    case MODE_BALLS:
      changeBallCount(clicks);
      break;
    case MODE_BOUNCE:
      bounciness = clampFix(bounciness + clicks * float2fix15(0.05), 0, int2fix15(2));
      break;
    case MODE_GRAVITY:
      gravity = clampFix(gravity + clicks * float2fix15(0.02), float2fix15(0.02), int2fix15(2));
      break;
    default:
      break;
  }

  // Any parameter change resets the stats
  resetStats();
}

// draw the ball
void drawBall(Ball *ball) {
  //drawCircle(fix2int15(ball->x), fix2int15(ball->y), BALL_RADIUS, BLUE);
  short x = (short)fix2int15(ball->x);
  short y = (short)fix2int15(ball->y);

  drawPixel(x + BALL_RADIUS, y, BLUE); // right
  drawPixel(x - BALL_RADIUS, y, BLUE); // left
  drawPixel(x, y + BALL_RADIUS, BLUE); // down
  drawPixel(x, y - BALL_RADIUS, BLUE); // up


}

// Draw the boundaries
void drawArena() {
  drawVLine(100, 100, 280, WHITE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}

// keep track of all the pegs coords in the pegs array 
void initPegs() {
  int peg_index = 0;

  for(int row = 0; row < NUM_ROWS; row++){
    int y = PEG_START_Y + row * ROW_SPACE;
    int start_x = 320 - (row * PEG_SPACE) / 2; // figure out where first peg of each row starts

    for(int col = 0; col <= row; col++){
      int x = start_x + col * PEG_SPACE;

      pegs[peg_index].x = int2fix15(x);
      pegs[peg_index].y = int2fix15(y);

      peg_index++;
      

    }
  }
}

// actually draw the pegs, going thru the peg array one by one
void drawPegs() {

  for(int i = 0; i < NUM_PEGS; i++){
    short x = (short)fix2int15(pegs[i].x);
    short y = (short)fix2int15(pegs[i].y);

    drawCircle(x, y, PEG_RADIUS, WHITE);
    // drawPixel(x + PEG_RADIUS, y, WHITE); // right
    // drawPixel(x - PEG_RADIUS, y, WHITE); // left
    // drawPixel(x, y + PEG_RADIUS, WHITE); // down
    // drawPixel(x, y - PEG_RADIUS, WHITE); // up
  }
  
}

void handlePegCollisions(Ball *ball) {

  // Which row is the ball closest to? Rows are 19 px apart and collisions
  // need < 10 px, so only the nearest row can be hit.
  int by = fix2int15(ball->y);
  int row = (by - PEG_START_Y + ROW_SPACE / 2) / ROW_SPACE;
  if (row < 0 || row >= NUM_ROWS) {
    //ball->last_peg = -1;
    return;
  }

  // Which peg in that row is closest horizontally?
  int bx = fix2int15(ball->x);
  int start_x = 320 - (row * PEG_SPACE) / 2;
  int col = (bx - start_x + PEG_SPACE / 2) / PEG_SPACE;
  if (col < 0) col = 0;
  if (col > row) col = row;

  // Row r starts at index r*(r+1)/2 in pegs[]
  int i = row * (row + 1) / 2 + col;

  fix15 dx = ball->x - pegs[i].x;
  fix15 dy = ball->y - pegs[i].y;

  // If we were touching a peg before, check whether we've moved away from it
  if (ball->last_peg >= 0) {
      fix15 last_dx = ball->x - pegs[ball->last_peg].x;
      fix15 last_dy = ball->y - pegs[ball->last_peg].y;

      fix15 release_distance =
          int2fix15(BALL_RADIUS + PEG_RADIUS + 2);

      // Once clearly outside the previous peg, allow another thunk later
      if (absfix15(last_dx) >= release_distance || absfix15(last_dy) >= release_distance) {
          ball->last_peg = -1;
      }
  }

  // cheap bounding check 
  if(absfix15(dx) >= COLLISION_DISTANCE || absfix15(dy) >= COLLISION_DISTANCE){
    return;
  }

  fix15 dist_squared = multfix15(dx, dx) + multfix15(dy, dy);



  // float dx_float = fix2float15(dx) ;
  // float dy_float = fix2float15(dy) ;

  // float distance = sqrt((dx_float * dx_float) + (dy_float * dy_float)) ;

  if ((dist_squared >= COLLISION_SQUARED) || dist_squared == 0) {
    //ball->last_peg = -1;
    return;
  }

  // WANNA DO UNNORMALIZED VECTOR CALCULATION HERE INSETAD 
  fix15 dot = multfix15(ball->vx, dx) + multfix15(ball->vy, dy);

  // only reflect if ball is moving INTO the peg 
  if(dot < 0){

    // factor = -2(v dot d) / (d dot d)
    fix15 factor = divfix(-2 * dot, dist_squared);

    // v' = v + factor * d 
    ball->vx += multfix15(factor, dx);
    ball->vy += multfix15(factor, dy);

    // Push ball just outside the peg 
    float dx_float = fix2float15(dx);
    float dy_float = fix2float15(dy);

    float distance = sqrt(dx_float * dx_float + dy_float * dy_float);

    fix15 normal_x = float2fix15(dx_float / distance);
    fix15 normal_y = float2fix15(dy_float / distance);

    fix15 push_distance = int2fix15(PEG_RADIUS + BALL_RADIUS + 1);

    ball->x = pegs[i].x + multfix15(normal_x, push_distance);
    ball->y = pegs[i].y + multfix15(normal_y, push_distance);

      // Only thunk and lose energy on a NEW peg
    if (i != ball->last_peg) {
      thunk();
      ball->vx = multfix15(bounciness, ball->vx);
      ball->vy = multfix15(bounciness, ball->vy);
      ball->last_peg = i;
    }

  }

  // fix15 intermediate_term =
  //     -2 * (multfix15(normal_x, ball->vx) + multfix15(normal_y, ball->vy));

  // // Move ball just outside the peg
  // ball->x = pegs[i].x + multfix15(normal_x, int2fix15(PEG_RADIUS + BALL_RADIUS + 1));
  // ball->y = pegs[i].y + multfix15(normal_y, int2fix15(PEG_RADIUS + BALL_RADIUS + 1));

  // // Reflect velocity off the peg
  // ball->vx = ball->vx + multfix15(normal_x, intermediate_term);
  // ball->vy = ball->vy + multfix15(normal_y, intermediate_term);

}

// used to be wallsAndEdges
void updateBallPos(Ball *ball){
  ball->x += ball->vx;
  ball->y += ball->vy;

  // If ball goes off left or right side, respawn at the top
  if (ball->x < int2fix15(BALL_RADIUS) || ball->x > int2fix15(640 - BALL_RADIUS)) {
      spawnBall(ball);
      return;
  }
  
  //fix15 collision_distance = int2fix15(BALL_RADIUS + PEG_RADIUS);

  handlePegCollisions(ball);

  // If ball reaches top of histogram, drop again from top
    // If ball reaches top of histogram, drop again from top
  if (ball->y >= int2fix15(HIST_TOP)) {
    int core = get_core_num();                       // 0 or 1
    histogram[core][bucketForX(fix2int15(ball->x))]++;
    ball_count[core]++;
    spawnBall(ball);
    return;
  }
  ball->vy += gravity;
}

// Detect wallstrikes, update velocity and position
// void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
// {
//   // Reverse direction if we've hit a wall
//   if (hitTop(*y)) {
//     *vy = (-*vy) ;
//     *y  = (*y + int2fix15(5)) ;
//   }
//   if (hitBottom(*y)) {
//     *vy = (-*vy) ;
//     *y  = (*y - int2fix15(5)) ;
//   } 
//   if (hitRight(*x)) {
//     *vx = (-*vx) ;
//     *x  = (*x - int2fix15(5)) ;
//   }
//   if (hitLeft(*x)) {
//     *vx = (-*vx) ;
//     *x  = (*x + int2fix15(5)) ;
//   } 
// }
//   // Update position using velocity
//   *x = *x + *vx ;
//   *y = *y + *vy ;

//   // Check for collision with peg
//   fix15 dx = *x - peg_x ;
//   fix15 dy = *y - peg_y ;

//   fix15 collision_distance = int2fix15(BALL_RADIUS + PEG_RADIUS) ;

//   if ((absfix15(dx) < collision_distance) &&
//       (absfix15(dy) < collision_distance)) {

//     float dx_float = fix2float15(dx) ;
//     float dy_float = fix2float15(dy) ;

//     float distance = sqrt((dx_float * dx_float) + (dy_float * dy_float)) ;

//     if ((distance < (BALL_RADIUS + PEG_RADIUS)) && (distance > 0)) {

//       fix15 normal_x = float2fix15(dx_float / distance) ;
//       fix15 normal_y = float2fix15(dy_float / distance) ;

//       fix15 intermediate_term =
//           -2 * (multfix15(normal_x, *vx) + multfix15(normal_y, *vy)) ;

//       // Move ball just outside the peg
//       *x = peg_x + multfix15(normal_x, int2fix15(PEG_RADIUS + BALL_RADIUS + 1)) ;
//       *y = peg_y + multfix15(normal_y, int2fix15(PEG_RADIUS + BALL_RADIUS + 1)) ;

//       // Change velocity so the ball bounces
//       *vx = *vx + multfix15(normal_x, intermediate_term) ;
//       *vy = *vy + multfix15(normal_y, intermediate_term) ;

//       thunk() ; // Play sound on collision

//       // Lose some energy during the bounce
//       *vx = multfix15(bounciness, *vx) ;
//       *vy = multfix15(bounciness, *vy) ;

//     }
//   }

//   // If ball falls off bottom of screen, drop again from top
//   if (*y > int2fix15(480)) {
//     spawnBoid(x, y, vx, vy, 0) ;
//     return ;
//   }

//     // Make gravity increase downward velocity every frame
//   *vy = *vy + gravity ;
// }





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
        // update boid color (color no longer used)
        // if ((user_input > 0) && (user_input < 16)) {
        //   color = (char)user_input ;
        // }
      } // END WHILE(1)
  PT_END(pt);
} // timer thread



// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // static because protothreads lose local variables across yields
    static char text[40];
    static uint32_t frame_start;
    static int spare_us = FRAME_US;

    PT_BEGIN(pt);

    // Start every ball at the top
    for (int i = 0; i < MAX_BALLS; i++) {
      spawnBall(&balls[i]);
    }

    while(1) {
      // Wait for the VGA driver to swap buffers (60 Hz)
      PT_YIELD_UNTIL(pt, draw_start_signal()) ;
      frame_start = time_us_32();

      // Apply button presses and encoder clicks
      handleInput();

      // Clear the buffer
      clearLowFrame(0, BLACK);
      sem_release(&draw_semaphore);   // wake core 1

      // On-screen info
      sprintf(text, "Mode: %s", mode_names[mode]);
      drawTextGLCD(10, 10, text, WHITE, BLACK);
      sprintf(text, "# of balls: %d", current_ball_count);
      drawTextGLCD(10, 20, text, WHITE, BLACK);
      sprintf(text, "Total balls: %u", (unsigned)(ball_count[0] + ball_count[1]));
      drawTextGLCD(10, 30, text, WHITE, BLACK);
      sprintf(text, "Bounciness: %.2f", fix2float15(bounciness));
      drawTextGLCD(10, 40, text, WHITE, BLACK);
      sprintf(text, "Gravity: %.2f", fix2float15(gravity));
      drawTextGLCD(10, 50, text, WHITE, BLACK);
      sprintf(text, "Time since boot (s): %d", (int)(time_us_64() / 1000000));
      drawTextGLCD(10, 60, text, WHITE, BLACK);
      sprintf(text, "Spare time (us): %d", spare_us);
      drawTextGLCD(10, 70, text, WHITE, BLACK);

      // Draw the pegs
      drawPegs();

      for (int i = 0; i < current_ball_count / 2; i++) {
        updateBallPos(&balls[i]);
        drawBall(&balls[i]);
      }

      PT_SEM_SDK_WAIT(pt, &done_semaphore);   // wait for core 1

      // Draw the histogram
      drawHistogram();

      // How much of the 1/60 s frame was left over? Negative = deadline missed.
      spare_us = FRAME_US - (int)(time_us_32() - frame_start);
      gpio_put(LED_PIN, spare_us < 0);
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// Animation on core 1

static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);


    while(1) {
      PT_SEM_SDK_WAIT(pt, &draw_semaphore) ;
      for (int i = current_ball_count / 2; i < current_ball_count; i++) {
        updateBallPos(&balls[i]);
        drawBall(&balls[i]);
      }
      sem_release(&done_semaphore);
    }
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
  printf("Hello World!\n");
  

  // initialize rotary encoder
  enc_init();

  // initialize LED that lights when a frame misses the 60 fps deadline
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, 0);

  // initialize VGA
  initVGA() ;

  // initialize pegs
  initPegs();

  dmaSetup();

  // Initialize the semaphore
  // Arguments: pointer to sem, initial count, max count
  sem_init(&draw_semaphore, 0, 1) ;
  sem_init(&done_semaphore, 0, 1) ;

  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // start core 1 
  //multicore_reset_core1();
  //multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);
  // pt_add_thread(protothread_anim1);  

  // start scheduler
  pt_schedule_start ;
} 
