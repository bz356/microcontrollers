/*
 * Lab 3: beam angle control
 * Based on PWM and IMU demo code by V. Hunter Adams (vha3@cornell.edu)
 *
 * The 1 kHz PWM interrupt reads the MPU6050, estimates the beam angle with a
 * complementary filter, and sets the motor PWM duty with P control.
 * The VGA shows the beam angle and the low-passed motor command. Serial
 * commands over UART0 set the requested angle and the PID gains.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 4 ---> PWM output
 *  - GPIO 0 ---> UART0 TX (to USB-serial adapter RX)
 *  - GPIO 1 ---> UART0 RX (to USB-serial adapter TX)
 *  - GPIO 14 ---> scope (toggles every PWM interrupt)
 *  - GPIO 15 ---> scope (toggles every VGA frame)
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - Pico 2 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - Pico 2 GND ---> MPU6050 GND
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "mpu6050.h"
#include "VGA/vga16_graphics_v3.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "pt_cornell_rp2040_v1_4.h"

// PWM wrap value and clock divide value
// For a CPU rate of 150 MHz, this gives
// a PWM frequency of 1 kHz.
#define WRAPVAL 5800
#define CLKDIV 25.0f

// GPIO we're using for PWM
#define PWM_OUT 4

// We will start drawing at column 81
#define LEFT_EDGE 81 

// Plot height in pixels and the value range each plot covers
#define PLOT_HEIGHT 150
#define ANGLE_MIN 0.0f
#define ANGLE_MAX 180.0f
#define CONTROL_MIN 0.0f
#define CONTROL_MAX ((float)WRAPVAL)

// Place the plots
#define TOP_PLOT_BOTTOM 230
#define BOTTOM_PLOT_BOTTOM 430
#define PLOT_WIDTH 530

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// Low-pass filtered accelerometer values
volatile fix15 filtered_ax;
volatile fix15 filtered_az;

// Arm angle convention: 0 = hanging straight down, 90 = sideways, 180 = up.
// Calibrate on the rig: RAW_ANGLE_AT_DOWN is the raw atan2 angle (0..360, what
// the old firmware displayed as "Actual") with the arm hanging straight down.
// ANGLE_DIRECTION is 1 if that raw angle increased as the arm swung up, -1 if
// it decreased.
#define RAW_ANGLE_AT_DOWN 90.0f
#define ANGLE_DIRECTION 1

// Angle estimates
volatile fix15 accel_angle;
volatile fix15 gyro_angle_delta;
volatile fix15 complementary_angle;
// Degrees in the arm convention above (0..180).
volatile float requested_angle = 90;

// PID gains, set from the serial console. Only kp is used so far; ki and kd
// are stored for the future I and D terms.
volatile float kp = 0.5f;
volatile float kerror = 0.0f;
volatile float ki = 0.5f;
volatile float kd = 0.0f;

#define PI 3.14159265358979323846f

// Accelerometer arm angle in degrees, wrapped to [-90, 270) so the arm's
// whole 0..180 travel is continuous (no 0/360 seam to confuse the filter).
static fix15 armAngleFromAccel(fix15 ax, fix15 az) {
    float raw = (float)((atan2(-ax, az) + PI) * 180.0 / PI);
    float arm = ANGLE_DIRECTION * (raw - RAW_ANGLE_AT_DOWN);
    while (arm < -90.0f) arm += 360.0f;
    while (arm >= 270.0f) arm -= 360.0f;
    return float2fix15(arm);
}

// Low-passed motor command, display only (the motor gets the raw control value)
volatile fix15 filtered_control = int2fix15(3000);

// Arrays for circular display buffer
fix15 angle_plot[PLOT_WIDTH];      // measured beam angle (deg)
float target_plot[PLOT_WIDTH];     // requested beam angle (deg)
fix15 control_plot[PLOT_WIDTH];    // low-passed motor command (0..WRAPVAL)

// Pointer to head of buffer
volatile int head = 0;

// character array
char screentext[40];

// draw speed: one plot sample every PLOT_INTERVAL PWM interrupts (1 ms each),
// so the 530-sample plot spans 530 * 10 ms = 5.3 s
#define PLOT_INTERVAL 10

// Some macros for max/min/abs
#define min(a,b) (((a)<(b)) ? (a):(b))
#define max(a,b) (((a)<(b)) ? (b):(a))
#define abs(a) (((a)>0) ? (a):-(a))

// Some parameters for PWM

uint slice_num;

void drawAxes() {
    // Draw the static aspects of the display
    setTextSize(1);
    setTextColor(WHITE);

    // Draw bottom plot: low-passed motor command
    drawHLine(75, 430, 5, CYAN);
    drawHLine(75, 355, 5, CYAN);
    drawHLine(75, 280, 5, CYAN);
    drawVLine(80, 280, 150, CYAN);
    sprintf(screentext, "Motor command (low-passed, 0-%d)", WRAPVAL);
    setCursor(LEFT_EDGE, 268);
    writeString(screentext);
    sprintf(screentext, "%d", WRAPVAL / 2);
    setCursor(45, 350);
    writeString(screentext);
    sprintf(screentext, "%d", WRAPVAL);
    setCursor(45, 280);
    writeString(screentext);
    sprintf(screentext, "0");
    setCursor(50, 425);
    writeString(screentext);

    // Draw top plot: beam angle
    drawHLine(75, 230, 5, CYAN);
    drawHLine(75, 155, 5, CYAN);
    drawHLine(75, 80, 5, CYAN);
    drawVLine(80, 80, 150, CYAN);
    setCursor(LEFT_EDGE, 68);
    setTextColor(WHITE);
    writeString("Beam angle (deg): measured");
    setTextColor(RED);
    writeString("  target");
    setTextColor(WHITE);
    sprintf(screentext, "90");
    setCursor(50, 150);
    writeString(screentext);
    sprintf(screentext, "180");
    setCursor(50, 75);
    writeString(screentext);
    sprintf(screentext, "0");
    setCursor(50, 225);
    writeString(screentext);
}

// Map a value in [min_v, max_v] to a pixel row in a plot, clamped so
// out-of-range values stay on the plot edge instead of drawing elsewhere.
static int plotY(int plot_bottom, float value, float min_v, float max_v) {
    float frac = (value - min_v) / (max_v - min_v);
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    return plot_bottom - (int)(PLOT_HEIGHT * frac);
}

static void drawStatusText(void) {
    // Take one snapshot of each value updated by the PWM interrupt on core 0.
    float angle_snapshot = fix2float15(complementary_angle);
    float target_snapshot = requested_angle;
    float command_snapshot = fix2float15(filtered_control);
    char status_text[60];

    setTextSize(2);
    setTextColor(WHITE);
    snprintf(status_text, sizeof(status_text), "Actual: %.1f deg  Target: %.1f deg",
             (double)angle_snapshot, (double)target_snapshot);
    setCursor(LEFT_EDGE, 4);
    writeString(status_text);
    snprintf(status_text, sizeof(status_text), "Command: %d/%d (%.1f%% duty)",
             (int)lroundf(command_snapshot), WRAPVAL,
             (double)(command_snapshot * 100.0f / WRAPVAL));
    setCursor(LEFT_EDGE, 24);
    writeString(status_text);
    snprintf(status_text, sizeof(status_text), "Kp: %.2f  Ki: %.2f  Kd: %.2f",
             (double)kp, (double)ki, (double)kd);
    setCursor(LEFT_EDGE, 44);
    writeString(status_text);
    setTextSize(1);
}

// PWM duty cycle
volatile int control = 3000;
volatile int old_control = 3000;

volatile int throttle = 10;

// P only: no accumulated error (I) or error-rate (D) term.
float doPid(float target_angle, float current_angle) {
    
    // Proportion
    float error = target_angle - current_angle;

    if (control <= 100 || control >= 5800) {
        kerror += error * 0.001f; // controller updates once every 1ms
    }
    

    return error * kp + kerror * ki;
}

// Command replies and telemetry share UART0, but use separate text buffers.
// Both writers run cooperatively on core 0; never hold this across serial_read.
static bool serial_tx_busy;
static struct pt pt_locked_serialout;
static PT_THREAD(protothread_locked_serialout(struct pt *pt)) {
    PT_BEGIN(pt);
    PT_YIELD_UNTIL(pt, !serial_tx_busy);
    serial_tx_busy = true;
    serial_write;
    serial_tx_busy = false;
    PT_END(pt);
}

#define serial_write_locked do { \
    PT_SPAWN(pt, &pt_locked_serialout, \
             protothread_locked_serialout(&pt_locked_serialout)); \
} while (0)

#define STATUS_INTERVAL_US 5000000ULL

// PWM interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));
    gpio_put(14, !gpio_get(14));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // Low-pass the accelerometer measurements before atan2
    filtered_ax = filtered_ax
                + ((acceleration[0] - filtered_ax) >> 4);

    filtered_az = filtered_az
                + ((acceleration[2] - filtered_az) >> 4);

    // Accelerometer angle (degrees - 15.16 fixed point), full atan2 (no small-angle approximation)
    accel_angle = armAngleFromAccel(filtered_ax, filtered_az);

    // Gyro angle delta (measurement times timestep) (15.16 fixed point),
    // flipped with ANGLE_DIRECTION so it matches the arm angle's sign
    gyro_angle_delta = multfix15(gyro[2], zeropt001) * ANGLE_DIRECTION;

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    /* ========================== PID Control +++++++++++++++++++*/
    float correction = doPid(
    requested_angle,
    fix2float15(complementary_angle)
    );

    // Replace 3000 with the PWM level that holds your desired position.
    float pwm_command = control + correction;

    // Keep the command within the valid PWM range.
    pwm_command = fminf((float)WRAPVAL, fmaxf(100.0f, pwm_command));
    control = (int)lroundf(pwm_command);
    /*========== P Control End ++++++++++++++++++++++++++++++*/

    // Low-pass the motor command for display, same 1/16 filter as the accel
    filtered_control = filtered_control
                     + ((int2fix15(control) - filtered_control) >> 2);
    
    // Increment drawspeed controller
    throttle += 1;
    // Maintain a circular buffer for display
    if ((throttle >= PLOT_INTERVAL)) {
        // Zero drawspeed controller
        throttle = 0;

        // Replace oldest data with new sample
        angle_plot[head] = complementary_angle;
        target_plot[head] = requested_angle;
        control_plot[head] = filtered_control;

        // Move the pointer to the oldest data
        head = ((head+1) == PLOT_WIDTH) ? 0 : (head + 1);

    }
    // Motor Code 
    // Update duty cycle
    if (control!=old_control) {
        old_control = control;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    }
}

static PT_THREAD (protothread_draw(struct pt *pt)) {
    PT_BEGIN(pt);

    static int start;
    static int i;

    // Draw axes
    drawAxes();
    drawStatusText();
    // Copy to other buffer
    copy_buffer_to_other();

    while(1) {
        // Wait for a buffer swap
        PT_YIELD_UNTIL(pt, draw_start_signal());
        gpio_put(15, !gpio_get(15));
        
        // Clear the buffer, draw the axes
        clearLowFrame(0, BLACK);
        drawAxes();
        drawStatusText();

        // Plot data oldest-->newest
        start = head;
        for (i = 0; i < PLOT_WIDTH; i++) {
            // Target first so the measured trace draws on top where they overlap
            drawPixel(LEFT_EDGE + i, plotY(TOP_PLOT_BOTTOM,
                target_plot[start], ANGLE_MIN, ANGLE_MAX), RED);
            drawPixel(LEFT_EDGE + i, plotY(TOP_PLOT_BOTTOM,
                fix2float15(angle_plot[start]), ANGLE_MIN, ANGLE_MAX), WHITE);
            drawPixel(LEFT_EDGE + i, plotY(BOTTOM_PLOT_BOTTOM,
                fix2float15(control_plot[start]), CONTROL_MIN, CONTROL_MAX), GREEN);
            start = ((start+1)==PLOT_WIDTH) ? 0 : (start+1);
        }
    }
    PT_END(pt);
}

// User input thread: independently set requested angle or PID gains.
static PT_THREAD (protothread_serial(struct pt *pt)) {
    PT_BEGIN(pt);
    static char classifier;
    static char extra_input;
    static float angle_in;
    static float gain_in;
    static volatile float *gain;
    while(1) {
        sprintf(pt_serial_out_buffer,
                "command (a = requested angle, p = Kp, i = Ki, d = Kd): ");
        serial_write_locked;
        serial_read;
        if (sscanf(pt_serial_in_buffer, " %c %c", &classifier, &extra_input) != 1) {
            sprintf(pt_serial_out_buffer, "enter a, p, i, or d\r\n");
            serial_write_locked;
            continue;
        }

        if (classifier == 'p' || classifier == 'i' || classifier == 'd') {
            gain = (classifier == 'p') ? &kp : (classifier == 'i') ? &ki : &kd;
            snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
                     "K%c (>= 0, currently %.3f): ", classifier, (double)*gain);
            serial_write_locked;
            serial_read;
            if (sscanf(pt_serial_in_buffer, "%f %c", &gain_in, &extra_input) == 1 &&
                isfinite(gain_in) && gain_in >= 0.0f) {
                *gain = gain_in;
                snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
                         "K%c set to %.3f\r\n", classifier, (double)gain_in);
            } else {
                sprintf(pt_serial_out_buffer, "invalid gain; enter a number >= 0\r\n");
            }
            serial_write_locked;
        } else if (classifier == 'a') {
            sprintf(pt_serial_out_buffer, "requested angle (0-180 degrees): ");
            serial_write_locked;
            serial_read;
            if (sscanf(pt_serial_in_buffer, "%f %c", &angle_in, &extra_input) == 1 &&
                isfinite(angle_in) && angle_in >= 0.0f && angle_in <= 180.0f) {
                requested_angle = angle_in;
                snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
                         "requested angle set to %.1f deg\r\n", (double)angle_in);
            } else {
                sprintf(pt_serial_out_buffer, "invalid angle; enter a number from 0 to 180\r\n");
            }
            serial_write_locked;
        } else {
            sprintf(pt_serial_out_buffer, "unknown command; enter a, p, i, or d\r\n");
            serial_write_locked;
        }
    }
    PT_END(pt);
}

// Independent of command input, so status still prints while serial_read waits.
static PT_THREAD(protothread_status(struct pt *pt)) {
    PT_BEGIN(pt);
    static uint64_t next_status_us;
    static char status_text[160];
    static unsigned send_index;
    static fix15 filtered_snapshot;
    static float requested_snapshot;
    static int duty_snapshot;
    static uint32_t irq_state;
    next_status_us = time_us_64() + STATUS_INTERVAL_US;

    while (1) {
        PT_YIELD_UNTIL(pt, time_us_64() >= next_status_us);
        // Only the snapshots need interrupts disabled, not formatting or UART I/O.
        irq_state = save_and_disable_interrupts();
        filtered_snapshot = complementary_angle;
        requested_snapshot = requested_angle;
        duty_snapshot = old_control; // Last value actually applied by the PWM ISR.
        restore_interrupts(irq_state);
        snprintf(status_text, sizeof(status_text),
                 "\r\nfiltered=%.1f deg  requested=%.1f deg  duty=%d/%d\r\n",
                 (double)fix2float15(filtered_snapshot), (double)requested_snapshot,
                 duty_snapshot, WRAPVAL);

        PT_YIELD_UNTIL(pt, !serial_tx_busy);
        serial_tx_busy = true;
        for (send_index = 0; status_text[send_index] != '\0'; ++send_index) {
            PT_YIELD_UNTIL(pt, uart_is_writable(UART_ID));
            uart_putc(UART_ID, status_text[send_index]);
        }
        serial_tx_busy = false;
        next_status_us += STATUS_INTERVAL_US;
        if (next_status_us <= time_us_64()) {
            next_status_us = time_us_64() + STATUS_INTERVAL_US;
        }
    }
    PT_END(pt);
}

// Entry point for core 1
void core1_entry() {
    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    gpio_put(15, 0);
    pt_add_thread(protothread_draw);
    pt_schedule_start;
}

int main() {
    set_sys_clock_khz(150000, true);

    // Initialize stdio
    stdio_init_all();
    printf("\nlab3 PWM demo booting\n");

    // Initialize VGA
    initVGA();

     ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    // Pins reset with pull-downs; without pull-ups SCL sits low if no IMU is attached.
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // MPU6050 initialization
    if (!mpu6050_reset()) {
        printf("MPU6050 did not answer on GPIO %d/%d\n", SDA_PIN, SCL_PIN);
    }
    mpu6050_read_raw(acceleration, gyro);

    // Initialize accelerometer filters
    filtered_ax = acceleration[0];
    filtered_az = acceleration[2];

    // Get initial absolute angle from accelerometer
    accel_angle = armAngleFromAccel(filtered_ax, filtered_az);

    // Start complementary filter at accelerometer estimate
    complementary_angle = accel_angle;
    requested_angle = fix2float15(complementary_angle);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO PWM_OUT that it is allocated to the PWM
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO PWM_OUT (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL);
    pwm_set_clkdiv(slice_num, CLKDIV);

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 3000);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    gpio_init(14);
    gpio_set_dir(14, GPIO_OUT);
    gpio_put(14, 0);
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_status);
    pt_schedule_start;
}
