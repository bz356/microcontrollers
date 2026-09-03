
/*
 * Lab 1: Synthesizing Birdsong
 *
 * Pinout:
 * GPIO 5 (pin 7) Chip select
 * GPIO 6 (pin 9) SCK/spi0_sclk
 * GPIO 7 (pin 10) MOSI/spi0_tx
 * GPIO 2 (pin 4) GPIO output for timing ISR
 * 3.3v (pin 36) -> VCC on DAC 
 * GND  (pin 3)  -> GND on DAC 
 * LDAC -> GND
 * ADC 0 (pin 26) -> Slide
 * 
 * Wire debugging pins and UART
 *
 * Week 1: 
 * Modified code to send waveform to other DAC output channel
 * and integrated ADC demo code with DDS tone generating code
 */


#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

// protothreads
#include "pt_cornell_rp2040_v1_4.h"

// ADC definitions
#define LED_PIN 25
#define ADC_PIN 26
#define ADC_MUX 0

// DAC definitions
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// DDS parameters
#define two32 4294967296.0 // 2^32 
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// DDS variables
volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = (800.0*two32)/Fs;

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];

// SPI definitions
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// SPI data
uint16_t DAC_data; // output value

unsigned int adc_val;

// Alarm interrupt setup
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

// GPIO to indicate ongoing ISR
#define ISR_GPIO 2


// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define LED             25

unsigned int keycodes[NUMKEYS] = {      0x57, 0x6E, 0x5E, 0x3E, 0x6D,
                                        0x5D, 0x3D, 0x6B, 0x5B, 0x3B,
                                        0x67, 0x37} ;
unsigned int scancodes[KEYROWS] = {   0xE, 0xD, 0xB, 0x7} ;
unsigned int button = 0x70 ;


char keytext[40];
int prev_key = 0;

// Alarm ISR
static void alarm_irq(void) {

    // assert GPIO to indicate beginning of interrupt
    gpio_put(ISR_GPIO, 1);

    // Clear alarm flag
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Schedules next alarm
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // scaling ADC value and updating phase_incr_main
    double freq = (adc_val / 4095.0) * 10000.0;
    phase_incr_main = (unsigned int)((freq * two32) / Fs);

	// DDS phase and sine table lookup
	phase_accum_main += phase_incr_main;
    DAC_data = (DAC_config_chan_A | ((sin_table[phase_accum_main>>24] + 2048) & 0xffff));

    // send SPI data
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

    // De-assert GPIO to indicate ending of interrupt
    gpio_put(ISR_GPIO, 0);

}



// ADC thread
static PT_THREAD (protothread_toggle25(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1) {
        // toggling GPIO
        gpio_put(LED_PIN, !gpio_get(LED_PIN));

        // reading and printing ADC value
        adc_val = adc_read();
        printf("ADC value: %d\n", adc_val);

        PT_YIELD_usec(1000);
    } 
    // every thread ends with PT_END(pt)
    PT_END(pt);
}

// This thread runs on core 0
static PT_THREAD (protothread_keypad(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int i ;
    static uint32_t keypad ;

    while(1) {

        gpio_put(LED, !gpio_get(LED)) ;

        // Scan the keypad!
        for (i=0; i<KEYROWS; i++) {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            // Small delay required
            sleep_us(1) ;
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            // Break if button(s) are pressed
            if ((~keypad) & button) break ;
        }
        // If we found a button . . .
        if ((~keypad) & button) {
            // Look for a valid keycode.
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ;
            }
            // If we don't find one, report invalid keycode
            if (i==NUMKEYS) (i = -1) ;
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else (i=-1) ;

        // Print key to terminal
        printf("\n%d", i) ;

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

int main(){
    // UART serial initialization
    stdio_init_all();
    printf("Hello, DAC!\n");
    printf("\n\rProtothreads RP2040 v1.4\n\r");

    // SPI configuration
    spi_init(SPI_PORT, 20000000);
    spi_set_format(SPI_PORT, 16, 0, 0, 0);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    // ADC configuration
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_MUX);

    // ISR indicator configuration
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    // LED configuration
    gpio_init(LED_PIN);  
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, true);

    // Build sin lookup table scaled to values between 0 and 4096
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = (int)(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Configure alarm 0 interrupt
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    irq_set_enabled(ALARM_IRQ, true);

    // Schedule first alarm
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    gpio_set_dir((BASE_KEYPAD_PIN+4), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN+5), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN+6), GPIO_IN);
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN)) ;
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_up((BASE_KEYPAD_PIN+4)) ;
    gpio_pull_up((BASE_KEYPAD_PIN+5)) ;
    gpio_pull_up((BASE_KEYPAD_PIN+6)) ;

  // protothread initialization
  pt_add_thread(protothread_toggle25);
  pt_add_thread(protothread_keypad) ;
  
  // scheduler initialization
  pt_schedule_start;
}