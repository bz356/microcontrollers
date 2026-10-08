
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
#include "hardware/vreg.h"
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
#define MAX_BALLS 55000
// Ball count at reset: tune to the largest value that keeps the LED off
#define START_BALLS 55000

typedef struct  Ball {
  fix15 x; 
  fix15 y;
  fix15 vx; 
  fix15 vy; 

  // uint8_t last_peg; // to keep track of when to "thunk"
} Ball;

// Compact storage: 8 bytes per ball (55,000 balls = 440 KB).
//   pos = x (bits 0-15, Q10.6 unsigned) | y (bits 16-31, Q9.6 signed)
//   vel = vx (bits 0-14, Q6.8 signed) | vy (bits 15-29, Q6.8 signed) | held (bits 30-31)
// Velocities saturate at +/-64 px/frame (a full-height fall at max gravity is ~38).
//
// "held" replaces the old per-ball last_peg byte. A held peg is always within
// 11 px of the ball (it's released as soon as the ball gets farther), so it must
// sit in one of the two rows bracketing the ball, and only one peg per row can be
// that close. 2 bits are enough: 0 = none, 1 = upper row, 2 = lower row, with the
// column recovered from x. See heldRowBase().
typedef struct PackedBall {
  uint32_t pos;
  uint32_t vel;
} PackedBall;

static PackedBall packed_balls[MAX_BALLS] __attribute__((aligned(8)));

// Shifts between fix15 (15 frac bits) and storage formats
#define POS_SHIFT 9   // 15 - 6
#define VEL_SHIFT 7   // 15 - 8
#define VEL_MAX   16383   // 15-bit signed limit, in Q6.8 units

// Round instead of truncate, so positions don't drift left over time
#define ROUND_SHIFT(v, s) (((v) + (1 << ((s) - 1))) >> (s))

// Unpack: move each field to the top of the word, then arithmetic-shift
// down so it lands already scaled to fix15 (no separate multiply)
#define UNPACK_X(p)  ((fix15)(((uint32_t)(p) << 16) >> (16 - POS_SHIFT)))
#define UNPACK_Y(p)  ((fix15)((int32_t)((p) & 0xFFFF0000u) >> (16 - POS_SHIFT)))
#define UNPACK_VX(v) ((fix15)((int32_t)((uint32_t)(v) << 17) >> (17 - VEL_SHIFT)))
#define UNPACK_VY(v) ((fix15)((int32_t)(((uint32_t)(v) << 2) & 0xFFFE0000u) >> (17 - VEL_SHIFT)))
#define UNPACK_HELD(v) ((uint32_t)(v) >> 30)

// Whole-pixel position as stored (what the held-peg code is relative to)
#define STORED_X_PX(p) ((int)(((p) & 0xFFFFu) >> 6))
#define STORED_Y_PX(p) ((int)((int32_t)(p) >> 22))

#define PACK_POS(x16, y16) (((uint32_t)(x16) & 0xFFFFu) | ((uint32_t)(y16) << 16))
#define PACK_VEL(vx15, vy15, held)   (((uint32_t)(vx15) & 0x7FFFu) | (((uint32_t)(vy15) & 0x7FFFu) << 15) | ((uint32_t)(held) << 30))

static inline int32_t satVel(int32_t v) {
  return (v > VEL_MAX) ? VEL_MAX : (v < -VEL_MAX - 1) ? -VEL_MAX - 1 : v;
}

static inline void loadBall(int i, Ball *b) {
  PackedBall p = packed_balls[i];
  b->x  = UNPACK_X(p.pos);
  b->y  = UNPACK_Y(p.pos);
  b->vx = UNPACK_VX(p.vel);
  b->vy = UNPACK_VY(p.vel);
}

// Stores with no held peg
static inline void storeBall(int i, const Ball *b) {
  packed_balls[i].pos = PACK_POS(ROUND_SHIFT(b->x, POS_SHIFT), ROUND_SHIFT(b->y, POS_SHIFT));
  packed_balls[i].vel = PACK_VEL(satVel(ROUND_SHIFT(b->vx, VEL_SHIFT)),
                                 satVel(ROUND_SHIFT(b->vy, VEL_SHIFT)), 0);
}

// Ball balls[MAX_BALLS];
int current_ball_count = START_BALLS;

#define NUM_ROWS 16
#define PEG_START_Y 60 // where the first peg starts
#define ROW_SPACE 19 // space between rows (vertical distance)
#define PEG_SPACE 38 // space between pegs 

#define NUM_PEGS 136 // 1 + 2 + 3... + 16 = 136


// Peg positions aren't stored: peg (row, col) is centered at
//   x = 320 - row * PEG_SPACE/2 + col * PEG_SPACE,  y = PEG_START_Y + row * ROW_SPACE
// and peg index = row_base[row] + col. Small tables (filled by initPegs) keep
// the per-ball loop free of divides.
#define PEG_CX(row, col) (320 - (row) * (PEG_SPACE / 2) + (col) * PEG_SPACE)
#define PEG_CY(row)      (PEG_START_Y + (row) * ROW_SPACE)

// Pixel y range where the nearest-row formula (y - 51) / 19 gives a valid row
// (C division truncates toward zero, so y = 33..50 also maps to row 0)
#define ROW_Y_MIN (PEG_START_Y - ROW_SPACE / 2 - ROW_SPACE + 1)
#define ROW_Y_MAX (PEG_START_Y - ROW_SPACE / 2 + NUM_ROWS * ROW_SPACE - 1)
#define ROW_TABLE_SIZE (ROW_Y_MAX - ROW_Y_MIN + 1)

static uint8_t row_of_y[ROW_TABLE_SIZE];   // nearest peg row for pixel y - ROW_Y_MIN
static int16_t row_left[NUM_ROWS];         // first peg x of the row minus PEG_SPACE/2
static uint8_t row_base[NUM_ROWS];         // index of the row's first peg

// Frame buffer pointer from the VGA driver (DMA rewrites it at each swap)
extern char * current_draw_buffer;

// Unaligned access types, so a pixel run that crosses a byte boundary is one RMW
typedef uint16_t __attribute__((aligned(1), may_alias)) u16_unaligned;
typedef uint32_t __attribute__((aligned(1), may_alias)) u32_unaligned;


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
#define PEG_RADIUS 7

#define COLLISION_DISTANCE int2fix15(BALL_RADIUS + PEG_RADIUS -1)
#define COLLISION_SQUARED  multfix15(COLLISION_DISTANCE, COLLISION_DISTANCE)

// 0.5 in fix15 = 2^14
#define HALF_FIX15 (1 << 14) 

fix15 peg_x = int2fix15(320) ;
fix15 peg_y = int2fix15(240) ;

fix15 gravity = float2fix15(0.37) ;
// fix15 bounciness = float2fix15(0.5) ;
fix15 bounciness = HALF_FIX15 ; 

int global_ctr_chan;
int global_data_chan;

// Create a semaphore
semaphore_t draw_semaphore ;
semaphore_t done_semaphore ;

// Both cores pull chunks of balls from a shared counter, so whichever core
// has less other work (core 0 also draws text/pegs/histogram) does more balls
#define BALL_CHUNK 512
static volatile int next_ball;
static spin_lock_t *work_lock;

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
        int raw_sin = (int)(2047 * sin((float)i*6.283/(float)sine_table_size) + 2047); //12 bit
        DAC_data[i] = DAC_config_chan_B | (raw_sin & 0x0fff) ;
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

void spawnBall(Ball *ball, int i) {
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
  (void)i;            // storeBall starts it with no held peg
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
    // spawnBall(&balls[i], i);
    Ball b;
    spawnBall(&b, i);
    storeBall(i, &b);
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
      changeBallCount(clicks*100);
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

  // drawPixel(x + BALL_RADIUS, y, BLUE); // right
  // drawPixel(x - BALL_RADIUS, y, BLUE); // left
  // drawPixel(x, y + BALL_RADIUS, BLUE); // down
  // drawPixel(x, y - BALL_RADIUS, BLUE); // up

  drawHLine(x - 1, y - 1, 2, BLUE);
  drawHLine(x - 1, y,     2, BLUE);


}

// Draw the boundaries
void drawArena() {
  drawVLine(100, 100, 280, WHITE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}

// Peg outline as one bitmask per row: bit k of peg_sprite[r] is pixel
// (x - PEG_RADIUS + k, y - PEG_RADIUS + r). Same shape drawCircle makes.
#define PEG_SIZE (2 * PEG_RADIUS + 1)
static uint16_t peg_sprite[PEG_SIZE];

static void buildPegSprite(void) {
  int r = PEG_RADIUS;
  int f = 1 - r, ddF_x = 1, ddF_y = -2 * r, x = 0, y = r;
  #define SPRITE_SET(px, py) (peg_sprite[(py) + r] |= (uint16_t)(1u << ((px) + r)))
  SPRITE_SET(0, r);  SPRITE_SET(0, -r);
  SPRITE_SET(r, 0);  SPRITE_SET(-r, 0);
  while (x < y) {
    if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
    x++; ddF_x += 2; f += ddF_x;
    SPRITE_SET( x,  y); SPRITE_SET(-x,  y); SPRITE_SET( x, -y); SPRITE_SET(-x, -y);
    SPRITE_SET( y,  x); SPRITE_SET(-y,  x); SPRITE_SET( y, -x); SPRITE_SET(-y, -x);
  }
  #undef SPRITE_SET
}

// keep track of all the pegs coords in the pegs array
void initPegs() {
  int peg_index = 0;

  for(int row = 0; row < NUM_ROWS; row++){
    int start_x = PEG_CX(row, 0); // where the first peg of each row starts

    row_left[row] = start_x - PEG_SPACE / 2;
    row_base[row] = peg_index;

    for(int col = 0; col <= row; col++){
      peg_index++;
    }
  }

  // Nearest row for every pixel y (exactly what the old per-ball divide gave,
  // including C's round-toward-zero; negative y never maps to a row)
  for (int y = ROW_Y_MIN; y <= ROW_Y_MAX; y++) {
    row_of_y[y - ROW_Y_MIN] = (y - PEG_START_Y + ROW_SPACE / 2) / ROW_SPACE;
  }

  buildPegSprite();
}

// actually draw the pegs: OR each sprite row into the frame with one 32-bit RMW
void drawPegs() {
  uint8_t *buf = (uint8_t *)current_draw_buffer;

  for(int row = 0; row < NUM_ROWS; row++){
    uint8_t *row_ptr = buf + 80 * (PEG_CY(row) - PEG_RADIUS);

    for(int col = 0; col <= row; col++){
      int px = PEG_CX(row, col) - PEG_RADIUS;   // sprite's left edge
      uint8_t *p = row_ptr + (px >> 3);
      int sh = px & 7;

      for (int r = 0; r < PEG_SIZE; r++, p += 80) {
        *(u32_unaligned *)p |= (uint32_t)peg_sprite[r] << sh;
      }
    }
  }
}

// ==================================================
// Fast per-ball update: load, step, store and draw in one pass.
// Everything here runs from RAM (__not_in_flash_func) so the hot loop
// never waits on the flash cache, which both cores share.
// ==================================================

#define RELEASE_DISTANCE int2fix15(BALL_RADIUS + PEG_RADIUS + 2)

// Per-core xorshift RNG: cheaper than rand(), and no shared state between cores
static uint32_t rng_state[2] = { 0x12345678u, 0x9e3779b9u };

static inline uint32_t fastRand(int core) {
  uint32_t s = rng_state[core];
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  rng_state[core] = s;
  return s;
}

// Slow path: only runs when the ball is inside the bounding box of peg i.
// Kept out of line so the common no-collision path stays small.
static void __no_inline_not_in_flash_func(resolvePegCollision)(Ball *ball, int i, int *last_peg,
                                                                fix15 dx, fix15 dy) {
  fix15 abs_dx = absfix15(dx);
  fix15 abs_dy = absfix15(dy);
  fix15 peg_cx = ball->x - dx;   // dx = ball - peg
  fix15 peg_cy = ball->y - dy;

  // alpha max + beta min (alpha = 1, beta = 1/4)
  fix15 max_d = (abs_dx > abs_dy) ? abs_dx : abs_dy;
  fix15 min_d = (abs_dx > abs_dy) ? abs_dy : abs_dx;
  fix15 approx_distance = max_d + (min_d >> 2);

  if (approx_distance >= COLLISION_DISTANCE || approx_distance == 0) return;

  fix15 dist_squared = multfix15(dx, dx) + multfix15(dy, dy);
  if (dist_squared == 0) return;

  fix15 dot = multfix15(ball->vx, dx) + multfix15(ball->vy, dy);
  if (dot >= 0) return;   // moving away from the peg

  // v' = v - 2(v.d)/(d.d) * d
  // Single-precision FPU divide (~14 cycles) instead of a 64-bit integer divide
  fix15 factor = (fix15)((float)(-2 * dot) * 32768.0f / (float)dist_squared);
  ball->vx += multfix15(factor, dx);
  ball->vy += multfix15(factor, dy);

  // Normalize with a scaled-down 32-bit divide (hardware SDIV)
  int32_t scaled_dx = dx >> 4;
  int32_t scaled_dy = dy >> 4;
  int32_t scaled_distance = approx_distance >> 4;
  if (scaled_distance == 0) return;

  fix15 normal_x = (fix15)((scaled_dx * (1 << 15)) / scaled_distance);
  fix15 normal_y = (fix15)((scaled_dy * (1 << 15)) / scaled_distance);

  // Push ball just outside the peg
  fix15 push_distance = int2fix15(PEG_RADIUS + BALL_RADIUS + 1);
  ball->x = peg_cx + multfix15(normal_x, push_distance);
  ball->y = peg_cy + multfix15(normal_y, push_distance);

  // Only thunk and lose energy on a NEW peg
  if (i != *last_peg) {
    thunk();
    if (bounciness == HALF_FIX15) {
      ball->vx >>= 1;
      ball->vy >>= 1;
    } else {
      ball->vx = multfix15(bounciness, ball->vx);
      ball->vy = multfix15(bounciness, ball->vy);
    }
    *last_peg = i;
  }
}

// Draw the 2x2 ball covering pixels (px-1..px, py-1..py).
// One unaligned 16-bit RMW per row handles the case where the two pixels
// straddle a byte boundary, so there's no branching on alignment.
static inline void plotBall(uint8_t *buf, int px, int py) {
  int x0 = px - 1;
  int y0 = py - 1;
  if ((unsigned)x0 > 638u || (unsigned)y0 > 478u) return;

  uint8_t *p = buf + 80 * y0 + (x0 >> 3);
  uint16_t m = (uint16_t)(3u << (x0 & 7));
  *(u16_unaligned *)p        |= m;
  *(u16_unaligned *)(p + 80) |= m;
}

// Upper of the two peg rows bracketing pixel y: floor((y - PEG_START_Y) / ROW_SPACE).
// Only called with y >= PEG_START_Y - ROW_SPACE (true within 12 px of any peg),
// so an unsigned divide by the constant is exact.
static inline int heldRowBase(int y_px) {
  return (int)((unsigned)(y_px - (PEG_START_Y - ROW_SPACE)) / ROW_SPACE) - 1;
}

// Update and draw balls [start, end) on the calling core
static void __not_in_flash_func(stepBalls)(int start, int end) {
  const int core = get_core_num();
  const fix15 g = gravity;            // read once per chunk, not once per ball
  int *hist = histogram[core];
  uint32_t fallen = 0;
  uint8_t *buf = (uint8_t *)current_draw_buffer;

  PackedBall *pb = &packed_balls[start];

  for (int n = end - start; n > 0; n--, pb++) {
    // Two 32-bit loads, unpacked straight into fix15 registers
    uint32_t pos = pb->pos;
    uint32_t vel = pb->vel;
    fix15 vx = UNPACK_VX(vel);
    fix15 vy = UNPACK_VY(vel);
    fix15 x  = UNPACK_X(pos) + vx;
    fix15 y  = UNPACK_Y(pos) + vy;

    // Held peg (-1 = none), decoded relative to the stored position
    int lp = -1, lrow = 0;
    uint32_t held = UNPACK_HELD(vel);
    if (held) {
      lrow = heldRowBase(STORED_Y_PX(pos)) + (int)held - 1;
      int lcol = (int)((unsigned)(STORED_X_PX(pos) - row_left[lrow]) / PEG_SPACE);
      lp = row_base[lrow] + lcol;

      // Release once clearly away from it
      fix15 ldx = x - int2fix15(PEG_CX(lrow, lcol));
      fix15 ldy = y - int2fix15(PEG_CY(lrow));
      if (absfix15(ldx) >= RELEASE_DISTANCE || absfix15(ldy) >= RELEASE_DISTANCE) {
        lp = -1;
      }
    }

    bool respawn = false;

    // x < BALL_RADIUS or x > 640 - BALL_RADIUS, as one unsigned compare
    if ((uint32_t)(x - int2fix15(BALL_RADIUS)) > (uint32_t)int2fix15(640 - 2 * BALL_RADIUS)) {
      respawn = true;   // off the side
    } else {
      // Nearest peg row from a table instead of a divide
      unsigned ty = (unsigned)(fix2int15(y) - ROW_Y_MIN);

      if (ty < ROW_TABLE_SIZE) {
        int row = row_of_y[ty];
        // Nearest peg in the row (same rounding as before, unsigned divide by constant)
        int t   = fix2int15(x) - row_left[row];
        int col = (t <= 0) ? 0 : (int)((unsigned)t / PEG_SPACE);
        if (col > row) col = row;

        // Cheap bounding check; most balls stop here
        fix15 dx = x - int2fix15(PEG_CX(row, col));
        fix15 dy = y - int2fix15(PEG_CY(row));
        if (absfix15(dx) < COLLISION_DISTANCE && absfix15(dy) < COLLISION_DISTANCE) {
          int p = row_base[row] + col;
          Ball b = { x, y, vx, vy };
          resolvePegCollision(&b, p, &lp, dx, dy);
          x = b.x;  y = b.y;  vx = b.vx;  vy = b.vy;
          if (lp == p) lrow = row;
        }
      }

      if (y >= int2fix15(HIST_TOP)) {
        hist[bucketForX(fix2int15(x))]++;
        fallen++;
        respawn = true;
      } else {
        vy += g;
      }
    }

    if (respawn) {
      x  = int2fix15(320);
      y  = 0;
      vx = (fix15)(fastRand(core) & 0xffff) - int2fix15(1);
      vy = 0;
      lp = -1;
    }

    // Pack back into 8 bytes
    int32_t x16 = ROUND_SHIFT(x, POS_SHIFT);
    int32_t y16 = ROUND_SHIFT(y, POS_SHIFT);
    uint32_t new_pos = PACK_POS(x16, y16);

    // Re-encode the held peg relative to the position just stored
    uint32_t new_held = 0;
    if (lp >= 0) {
      new_held = (uint32_t)(lrow - heldRowBase(STORED_Y_PX(new_pos)) + 1);
      if (new_held > 2) new_held = 0;   // can't happen while held; fail safe = release
    }

    pb->pos = new_pos;
    pb->vel = PACK_VEL(satVel(ROUND_SHIFT(vx, VEL_SHIFT)), satVel(ROUND_SHIFT(vy, VEL_SHIFT)), new_held);

    plotBall(buf, fix2int15(x), fix2int15(y));
  }

  ball_count[core] += fallen;   // one shared-memory write per chunk instead of per ball
}

// Claim chunks of balls until none are left. Called on both cores each frame.
static void __not_in_flash_func(runBallChunks)(void) {
  const int count = current_ball_count;

  while (1) {
    uint32_t save = spin_lock_blocking(work_lock);
    int start = next_ball;
    next_ball = start + BALL_CHUNK;
    spin_unlock(work_lock, save);

    if (start >= count) break;
    int end = start + BALL_CHUNK;
    if (end > count) end = count;
    stepBalls(start, end);
  }
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
      // spawnBall(&balls[i], i);
      Ball b;
      spawnBall(&b, i);
      storeBall(i, &b);
    }

    while(1) {
      // Wait for the VGA driver to swap buffers (60 Hz)
      PT_YIELD_UNTIL(pt, draw_start_signal()) ;
      frame_start = time_us_32();

      // Apply button presses and encoder clicks
      handleInput();

      // Clear the buffer
      clearLowFrame(0, BLACK);

      // Pegs go in before core 1 starts, so the two cores never
      // read-modify-write the same peg bytes at the same time
      drawPegs();

      // Reset the shared ball counter, then wake core 1
      next_ball = 0;
      sem_release(&draw_semaphore);

      // On-screen info (integer formatting: no float printf)
      sprintf(text, "Mode: %s", mode_names[mode]);
      drawTextGLCD(10, 10, text, WHITE, BLACK);
      sprintf(text, "# of balls: %d", current_ball_count);
      drawTextGLCD(10, 20, text, WHITE, BLACK);
      sprintf(text, "Total balls: %u", (unsigned)(ball_count[0] + ball_count[1]));
      drawTextGLCD(10, 30, text, WHITE, BLACK);
      {
        int b100 = (bounciness * 100 + HALF_FIX15) >> 15;   // value x 100, rounded
        int g100 = (gravity    * 100 + HALF_FIX15) >> 15;
        sprintf(text, "Bounciness: %d.%02d", b100 / 100, b100 % 100);
        drawTextGLCD(10, 40, text, WHITE, BLACK);
        sprintf(text, "Gravity: %d.%02d", g100 / 100, g100 % 100);
        drawTextGLCD(10, 50, text, WHITE, BLACK);
      }
      sprintf(text, "Time since boot (s): %d", (int)(time_us_64() / 1000000));
      drawTextGLCD(10, 60, text, WHITE, BLACK);
      sprintf(text, "Spare time (us): %d", spare_us);
      drawTextGLCD(10, 70, text, WHITE, BLACK);

      // Help core 1 with the balls until they're all claimed
      runBallChunks();

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
      runBallChunks();
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
  // set_sys_clock_khz(150000, true) ;
  
  vreg_disable_voltage_limit();
  vreg_set_voltage(VREG_VOLTAGE_1_50);
  sleep_ms(10);                         
  if (!set_sys_clock_khz(400000, false)) {   // false = return instead of assert
    set_sys_clock_khz(250000, true);         // fall back to a known-good clock
  }
  sleep_ms(1000);
  

  // initialize stio
  stdio_init_all() ;
  sleep_ms(5000);
  printf("Hello World!\n");
  printf("vreg = %d, clk = %lu\n", vreg_get_voltage(), clock_get_hz(clk_sys));

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

  // Hardware spinlock guarding the shared ball counter
  work_lock = spin_lock_instance(spin_lock_claim_unused(true));

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
