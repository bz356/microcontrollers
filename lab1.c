
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
 * ADC 1 (GPIO 27, pin 32) -> Slide
 * 
 * Wire debugging pins and UART
 *
 * Week 1: 
 * Modified code to send waveform to other DAC output channel
 * and integrated ADC demo code with DDS tone generating code
 */

////////////////////////////////// INCLUDES ///////////////////////////////////
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
#include <stdbool.h>

// protothreads
#include "pt_cornell_rp2040_v1_4.h"

////////////////////////////////// ADC / LED VARIABLES ///////////////////////////////////
#define LED_PIN 25
#define LED     25
#define ADC_PIN 27
#define ADC_MUX 1


// Slider value (0-4095) the ISR uses to set the output frequency
volatile unsigned int adc_val;

////////////////////////////////// SPI / DAC VARIABLES ///////////////////////////////////
// SPI definitions
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// SPI data
uint16_t DAC_data; // output value

/////////////////////////////////// DDS VARIABLES ////////////////////////////////////////
#define two32 4294967296.0 // 2^32 
#define Fs 200000
#define DELAY 5 // 1/Fs (in microseconds)

// DDS variables
volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = (800.0*two32)/Fs;

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];


////////////////////////////// TIMER ALARM / ISR VARIABLES ///////////////////////////////
// Alarm interrupt setup
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

// GPIO to indicate ongoing ISR (high while inside the ISR)
#define ISR_GPIO 2

////////////////////////////////// KEYPAD VARIABLES //////////////////////////////////////
#define BASE_KEYPAD_PIN 9 // rows on GPIO 9-12, columns on GPIO 13-15
#define KEYROWS         4
#define NUMKEYS         12

// Keypad lookup tables 
// 0-9 = digit keys, 10 = *, 11 = #
unsigned int keycodes[NUMKEYS] = {      0x57, 0x6E, 0x5E, 0x3E, 0x6D,
                                        0x5D, 0x3D, 0x6B, 0x5B, 0x3B,
                                        0x67, 0x37} ;
unsigned int scancodes[KEYROWS] = {   0xE, 0xD, 0xB, 0x7} ;
unsigned int button = 0x70 ;

// Enum defining the Debounce states 
typedef enum DebounceState{
    NOT_PRESSED, // no key pressed
    MAYBE_PRESSED, // key detected, waiting to confirm it isn't a bounce
    MAYBE_NOT_PRESSED, // key seems released, waiting to confirm
    PRESSED // key press confirmed
} DebounceState;

DebounceState debounce_state = NOT_PRESSED;

// Most recently detected keypad key (0-9 = digits, 10=*, 11=#)
int current_key = 0;

///////////////////////////// RECORDING / PLAYBACK VARIABLES /////////////////////////////
#define SAMPLING_FREQUENCY  100
#define TIME_SAMPLE 10000
#define MAX_SAMPLES         10000
#define PLAYBACK_FREQUENCY 100
#define PLAYBACK_TIME 1000

// One recording buffer per key 1-9. Each stores slider values (frequencies) not waveforms
uint16_t recording_one[MAX_SAMPLES];
uint16_t recording_two[MAX_SAMPLES];
uint16_t recording_three[MAX_SAMPLES];
uint16_t recording_four[MAX_SAMPLES];
uint16_t recording_five[MAX_SAMPLES];
uint16_t recording_six[MAX_SAMPLES];
uint16_t recording_seven[MAX_SAMPLES];
uint16_t recording_eight[MAX_SAMPLES];
uint16_t recording_nine[MAX_SAMPLES];

// Enum defining the Recording states
typedef enum RecordingState{
    NOT_RECORDED, // nothing recorded to this key
    RECORDING,    // key is currently being recorded
    RECORDED,     // key has a recording and is idle
    PLAYBACK      // key's recording is currently playing
} RecordingState;

// Recording state for each key, indexed by key number
RecordingState recording_state[15];

bool recording = false; // true while in record mode (toggled by *)
bool zero_pressed = true; // true when the live tone is on (toggled by 0)

// next write position in the current recording
uint16_t recording_index = 0; 

// next read position in the current playback
uint16_t playback_index = 0;

// number of samples recorded to each key
uint16_t recording_lengths[10] = {0};

uint16_t *recording_buf; // points to the buffer being recorded to
uint16_t *playback_buf;  // points to the buffer being played back

////////////////////////////////// COMPOSE MODE VARIABLES ////////////////////////////////
uint8_t composer_sequence[50] = {0}; // sequence of keys to play back in order
bool compose_mode = false; // true after # is pressed once
uint8_t compose_index = 0; // number of keys stored in the sequence

// Compose Playback Variables
bool compose_playback = false; // true while sequence is playing
uint8_t compose_playback_index = 0; // position in the sequence being played


////////////////////////////////// Alarm ISR ///////////////////////////////////
static void alarm_irq(void) {

    // Assert GPIO to indicate beginning of interrupt
    gpio_put(ISR_GPIO, 1);

    // Clear alarm flag
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Schedules next alarm
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Scaling ADC value and updating phase_incr_main
    double freq = (adc_val / 4095.0) * 10000.0;
    phase_incr_main = (unsigned int)((freq * two32) / Fs);

	// DDS phase and sine table lookup
	phase_accum_main += phase_incr_main;
    DAC_data = (DAC_config_chan_A | ((sin_table[phase_accum_main>>24] + 2048) & 0xffff));

    // Send SPI data to the DAC
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

    // De-assert GPIO to indicate ending of interrupt
    gpio_put(ISR_GPIO, 0);

}

////////////////////////////////// SLIDER / RECORD THREAD //////////////////////////////////
static PT_THREAD (protothread_slider_record(struct pt *pt))
{
    
    PT_BEGIN(pt);
    static uint64_t previous_time = 0; // time of the last recorded sample
    while(1) {
        // Pause while a recording is playing so the slider doesn't 
        // overwrite adc_val during playback
        PT_YIELD_UNTIL(pt, recording_state[current_key] != PLAYBACK);

        // Toggling the LED 
        gpio_put(LED_PIN, !gpio_get(LED_PIN));

        // Reading and printing ADC value
        unsigned int temp_val = adc_read();

        // Checking if 0 is pressed to enable/disable sound
        // Only pass slider value to the ISR if the tone is on or if a key is being recorded
        if (zero_pressed || recording_state[current_key] == PLAYBACK || recording_state[current_key] == RECORDING) {
            adc_val = temp_val; 
        }
        else {
            adc_val = 0;
        }
        
        // Check if 10 ms have passed since the last recorded sample
        bool sample = (time_us_64() >= previous_time + TIME_SAMPLE);

        // If a number key is being recorded, store slide value at 100Hz
        if (recording_state[current_key] == RECORDING && sample && current_key != 0) {
            previous_time = time_us_64();
            
            // Store only if buffer is valid and has space 
            if (recording_buf != NULL && recording_index < MAX_SAMPLES) {
                recording_buf[recording_index++] = temp_val ;
            }
        }
        // printf("ADC value: %d\n", adc_val);

        // Run again in ~0.1 ms (keeps frequency updates smooth)
        PT_YIELD_usec(100);
    } 
    // every thread ends with PT_END(pt)
    PT_END(pt);
}

////////////////////////////////// PLAYBACK THREAD //////////////////////////////////
// Feeds a key's recorded slider values to the ISR every 1 ms (10x the 100Hz recording rate)
// In Compose mode, moves on to the next key in the sequence when each recording finishes 
static PT_THREAD (protothread_play(struct pt *pt))
{
    PT_BEGIN(pt);
    static uint64_t previous_time = 0; // time of the last played sample
    while (1) {
        // Wait until a valid recorded key enters PLAYBACK mode
        PT_YIELD_UNTIL(pt, recording_state[current_key] == PLAYBACK && 
            current_key != 0 && 
            current_key >= 1 &&
            current_key <= 9 
        );
        
        
        // Check if 1 ms has passed since the last played sample
        bool playback = (time_us_64() >= previous_time + PLAYBACK_TIME);

        // Checking if it's time for the next sample to be played back 
        // Select the buffer for the current key 
        if (recording_state[current_key] == PLAYBACK && playback && current_key != 0) {
            previous_time = time_us_64();
            switch (current_key) {
                case 1: playback_buf = recording_one   ; break ;
                case 2: playback_buf = recording_two   ; break ;
                case 3: playback_buf = recording_three ; break ;
                case 4: playback_buf = recording_four  ; break ;
                case 5: playback_buf = recording_five  ; break ;
                case 6: playback_buf = recording_six   ; break ;
                case 7: playback_buf = recording_seven ; break ;
                case 8: playback_buf = recording_eight ; break ;
                case 9: playback_buf = recording_nine  ; break ;
                default: playback_buf = NULL ; break ; 
            }
            // Send the next saved sample to the synthesis ISR 
            if (playback_buf != NULL && playback_index <= recording_lengths[current_key]) {
                adc_val = playback_buf[playback_index++] ;
            }

            // End playback once all recorded samples have been used 
            else {
                recording_state[current_key] = RECORDED;
                playback_index = 0;
                printf("Finished playback\n");
                // In compose playback, start the next key in the sequence 
                if (compose_playback) {
                    compose_playback_index++;
                    if (compose_playback_index < compose_index) {
                        current_key = composer_sequence[compose_playback_index];
                        recording_state[current_key] = PLAYBACK;
                    } else {
                        // sequence done, don't run into the zero padding
                        compose_playback = false;
                        compose_playback_index = 0;
                        // leave current_key on the last real key so state[0] stays clean
                    }
                }
        }

        PT_YIELD_usec(1000);
        // every thread ends with PT_END(pt)
        
        }
    PT_END(pt);
    }
}


////////////////////////////////// KEYPAD THREAD //////////////////////////////////
// Scans the keypad, debounces presses/releases, and handles mode changes
// This thread runs on core 0
static PT_THREAD (protothread_keypad(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int i ; // loop index / detected key number
    static uint32_t keypad ; // raw row + column bits read from keypad
    static uint32_t possible ; // key code being confirmed by the debouncer

    while(1) {

        gpio_put(LED, !gpio_get(LED)) ;

        // Scan the keypad!
        for (i=0; i<KEYROWS; i++) {
            // Set a row low
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            // Small delay required
            sleep_us(1) ;
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            // Break if button(s) are pressed
            if ((~keypad) & button) break ;
        }

        /////////////// Debounce state machine ///////////////
        switch (debounce_state) {

            case NOT_PRESSED:
                // Key detected: remember it and wait to confirm 
                if((~keypad) & button){
                    debounce_state = MAYBE_PRESSED;
                    possible = keypad;
                }
                
                break;

            case MAYBE_PRESSED:
                sleep_ms(50);
                if(keypad == possible){
                    
                    
                    // Look for a valid keycode.
                    for (i=0; i<NUMKEYS; i++) {
                        if (possible == keycodes[i]) break ;
                    }
                    current_key = i;
                    
                    // Toggle record mode: ('*' = 10) is pressed
                    if (recording && i == 10) {
                        printf("STOPPED RECORDING\n");
                        recording = false;
                    }
                    else if (i == 10) {
                        printf("RECORDING\n");
                        recording = true;
                    }

                    // Begin recording if Record Mode is activated 
                    bool valid_record = (i>=1 && i<=9);
                    if (recording && valid_record) { 
                        recording_index = 0;
                        recording_state[current_key] = RECORDING;
                        // Point recording_buf at this key's buffer
                        switch (current_key) {
                            case 1: recording_buf = recording_one   ; break ;
                            case 2: recording_buf = recording_two   ; break ;
                            case 3: recording_buf = recording_three ; break ;
                            case 4: recording_buf = recording_four  ; break ;
                            case 5: recording_buf = recording_five  ; break ;
                            case 6: recording_buf = recording_six   ; break ;
                            case 7: recording_buf = recording_seven ; break ;
                            case 8: recording_buf = recording_eight ; break ;
                            case 9: recording_buf = recording_nine  ; break ;
                            default: recording_buf = NULL ; break ; 
                        }
                        // memset(recording_buf, 0, MAX_SAMPLES * sizeof(*recording_buf));   
                        printf("Started recording key: %d\n", i);
                    }

                    // If we don't find one, report invalid keycode
                    if (i==NUMKEYS) (i = -1) ;
                    debounce_state = PRESSED;
                    // printf("KEYPAD: %d\n", i) ;
                }else{
                    // If it was a bounce, go back to a NOT PRESSED state and waiting
                    debounce_state = NOT_PRESSED;
                }
                
                break;

            case PRESSED:
                // Key held down; wait for the reading to change
                if(keypad != possible){
                    debounce_state = MAYBE_NOT_PRESSED;
                }
                
                break;

            case MAYBE_NOT_PRESSED: // Key could have been released
                sleep_ms(50);
                // Look up which key was pressed
                for (i=0; i<NUMKEYS; i++) {
                    if (possible == keycodes[i]) break ;
                }

                // Release was just a bounce
                if(keypad == possible){
                    debounce_state = PRESSED;
                }

                // Release is confirmed: most key actions happen here
                else {

                    bool valid_record = (i>=1 && i<=9);

                    // FINISH RECORDING: releasing the key stops recording
                    // exits Record mode
                    if (recording && valid_record) {
                        printf("Stopped recording key: %d\n", i);
                        if (recording_state[current_key] == RECORDING) {

                            // Remember length of recording
                            recording_state[current_key] = RECORDED;
                            recording_lengths[current_key] = recording_index;
                        }

                        recording = false;
                        
                    }

                    // START PLAYBACK: releasing a key that has a recording plays it
                    else if (valid_record && recording_state[current_key] == RECORDED) {
                        playback_index = 0;
                        recording_state[current_key] = PLAYBACK;
                        // In Compose mode, also add this key to the sequence 
                        if (compose_mode) composer_sequence[compose_index++] = current_key;

                        // Point playback_buf at this key's buffer
                        switch (current_key) {
                            case 1: playback_buf = recording_one   ; break ;
                            case 2: playback_buf = recording_two   ; break ;
                            case 3: playback_buf = recording_three ; break ;
                            case 4: playback_buf = recording_four  ; break ;
                            case 5: playback_buf = recording_five  ; break ;
                            case 6: playback_buf = recording_six   ; break ;
                            case 7: playback_buf = recording_seven ; break ;
                            case 8: playback_buf = recording_eight ; break ;
                            case 9: playback_buf = recording_nine  ; break ;
                            default: playback_buf = NULL ; break ; 
                        }

                        printf("Started playback\n");

                    }

                    // Key 0: toggle the live tone on/off
                    else if (current_key == 0) {
                        zero_pressed = !zero_pressed;
                    }

                    // # is pressed for the first time: enter Compose mode
                    else if (current_key == 11 && !compose_mode) {
                        compose_mode = true;
                        printf("Composer mode on\n");
                    } 

                    // # is pressed again: play back the composed sequence
                    // starting with first key
                    else if (current_key == 11) {
                        printf("Composer mode sequence playback\n");
                        compose_playback = true;
                        compose_playback_index = 0;
                        current_key = composer_sequence[compose_playback_index];
                        recording_state[current_key] = PLAYBACK;
                    } 
                    debounce_state = NOT_PRESSED;
                }


                break;

        default:
            // optional safety case
            debounce_state = NOT_PRESSED;
            break;
    }
        

        PT_YIELD_usec(100) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

////////////////////////////////// MAIN ///////////////////////////////////
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

    // Build sin lookup table
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
    // Set all output pins to high
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN)) ;
    // Turn on pullup resistors for column pins
    gpio_pull_up((BASE_KEYPAD_PIN+4)) ;
    gpio_pull_up((BASE_KEYPAD_PIN+5)) ;
    gpio_pull_up((BASE_KEYPAD_PIN+6)) ;

  // protothread initialization
  pt_add_thread(protothread_slider_record);
  pt_add_thread(protothread_keypad) ;
  pt_add_thread(protothread_play);
  
  // scheduler initialization
  pt_schedule_start;
}