/*
 * Lab 3: beam angle control
 * Based on PWM and IMU demo code by V. Hunter Adams (vha3@cornell.edu)
 *
 * The 1 kHz PWM interrupt reads the MPU6050, estimates the beam angle with a
 * complementary filter, and sets the motor PWM duty with PID control.
 * The VGA shows the beam angle and the low-passed motor command. Serial
 * commands over UART0 set the requested angle and the PID gains.
 * Holding the demo button turns the motor off; releasing it runs the
 * timed angle sequence 0 -> 30 -> -30 -> 0 degrees (5 s apart).
 * Serial 't' toggles the settling-time timer: each new target is timed until
 * the beam stays within a band around it, marked on the plot with a yellow
 * (step) and magenta (settled) vertical line, and printed over serial.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 4 ---> PWM output
 *  - GPIO BUTTON_PIN ---> pushbutton ---> GND (internal pull-up, active low)
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

// PWM period is independent of the allowed motor command range.
// 150 MHz / (25 * (5999 + 1)) = 1 kHz, or 1 ms per update.
#define WRAPVAL 5999
#define CLKDIV 25.0f
#define CONTROL_DT 0.001f

// PID control parameters (PWM counts; angle error is in degrees,
// angular rate for the D term is in deg/s).
#define STARTUP_OFFSET 3000
#define KP 1.0f
#define KI 1.0f
#define KD 1.0f
#define CONTROL_MIN 1000.0f
#define CONTROL_MAX 3500.0f

// GPIO we're using for PWM
#define PWM_OUT 4

// Demo-sequence pushbutton: wire between this GPIO and GND (active low,
// internal pull-up). Change this to whichever pin the button ends up on.
#define BUTTON_PIN 26

// Demo sequence targets (degrees, -90 = hanging down, 0 = horizontal)
// and the time between steps.
#define SEQ_DOWN -90.0f
#define SEQ_HORIZONTAL 0.0f
#define SEQ_ABOVE 30.0f    // ~30 degrees above horizontal
#define SEQ_BELOW -30.0f   // ~30 degrees below horizontal
#define SEQ_STEP_US 5000000ULL

// Button polling period and how many identical reads count as a stable press
#define BUTTON_POLL_US 10000
#define BUTTON_STABLE_READS 2   // 2 reads x 10 ms = 20 ms debounce

// We will start drawing at column 81
#define LEFT_EDGE 81 

// Plot height in pixels and the value range each plot covers
#define PLOT_HEIGHT 150
#define ANGLE_MIN -90.0f
#define ANGLE_MAX 90.0f

// Place the plots
#define TOP_PLOT_BOTTOM 230
#define BOTTOM_PLOT_BOTTOM 430
// Nearly the full 640 px screen width (81 + 556 = 637)
#define PLOT_WIDTH 556

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// Low-pass filtered accelerometer values
volatile fix15 filtered_ax;
volatile fix15 filtered_az;

// Arm angle convention (matches the lab spec): -90 deg (-1.57 rad) = hanging
// straight down, 0 = horizontal, 90 deg (1.57 rad) = straight up.
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
// Degrees in the arm convention above (-90..90). Starts at horizontal.
volatile float requested_angle = 0;

// True while the demo button is held: the ISR forces the PWM to 0 and
// clears the integral so the sequence starts from a clean state.
volatile bool motor_off = false;

// True while the timed demo sequence is running. A serial angle command
// clears it so the user's angle is not overwritten by the next step.
volatile bool sequence_active = false;

// TEMP: serial stand-ins for the demo button until a physical one is wired.
// The serial thread sets these and the button thread consumes them, so all
// sequence state is still owned by protothread_button.
//   'x' -> same as pressing the button (motor off, beam hangs down)
//   's' -> same as releasing it (motor on, start the 15 s sequence)
volatile bool press_req = false;
volatile bool release_req = false;

// PID gains, set from the serial console.
volatile float kp = KP;
volatile float kerror = 0.0f;
volatile float ki = KI;
volatile float kd = KD;

#define PI 3.14159265358979323846f
#define DEG_TO_RAD (PI / 180.0f)

// Accelerometer arm angle in degrees, wrapped to [-180, 180) so the arm's
// whole -90..90 travel is continuous (no seam to confuse the filter).
static fix15 armAngleFromAccel(fix15 ax, fix15 az) {
    float raw = (float)((atan2(-ax, az) + PI) * 180.0 / PI);
    float arm = ANGLE_DIRECTION * (raw - RAW_ANGLE_AT_DOWN) - 90.0f;
    while (arm < -180.0f) arm += 360.0f;
    while (arm >= 180.0f) arm -= 360.0f;
    return float2fix15(arm);
}

// Low-passed motor command, display only (the motor gets the raw control value)
volatile fix15 filtered_control = int2fix15(STARTUP_OFFSET);

// Arrays for circular display buffer
fix15 angle_plot[PLOT_WIDTH];      // measured beam angle (deg)
float target_plot[PLOT_WIDTH];     // requested beam angle (deg)
fix15 control_plot[PLOT_WIDTH];    // low-passed motor command (0..WRAPVAL)

// Pointer to head of buffer
volatile int head = 0;

// Plot modes:
//  - rolling (default): circular buffer, newest sample at the right edge
//  - sweep: started when the demo sequence starts. The buffer is restarted
//    so t = 0 is the left edge and fills left to right; once the screen is
//    full (20 s) it switches back to rolling automatically. Pressing the
//    button, 'x', or a typed angle also returns to rolling early.
volatile bool sweep_mode = false;
volatile bool sweep_start_req = false;  // set by button thread, consumed by ISR
volatile int sweep_count = 0;           // samples recorded since sweep start

// Settling-time timer, toggled with the serial 't' command.
// While it is on, every change of the requested angle (a typed 'a' angle or
// a demo-sequence step) starts a timer in the PWM ISR, so it has 1 ms
// resolution. The beam counts as settled once it enters
// +/- SETTLE_BAND_DEG of the target and stays there for SETTLE_HOLD_MS. The
// reported settling time is from the step to the moment it ENTERED the band
// for good (the hold only confirms it). Gives up after SETTLE_TIMEOUT_MS.
// The plot marks the step with a YELLOW vertical line and the settle point
// with a MAGENTA one labeled with the time; the result also prints on serial.
#define SETTLE_BAND_DEG 2.0f
#define SETTLE_HOLD_MS 500      // must stay > PLOT_INTERVAL (see marker code)
#define SETTLE_TIMEOUT_MS 10000

volatile bool timer_mode = false;

// One marker flag per plot sample, so the lines scroll with the trace.
#define MARK_NONE 0
#define MARK_START 1
#define MARK_SETTLE 2
volatile uint8_t marker_plot[PLOT_WIDTH];
volatile uint16_t marker_ms[PLOT_WIDTH];   // settling time for MARK_SETTLE

// Timer state, owned by the ISR
static bool timing = false;
static float last_target = 0.0f;   // requested angle seen on the previous ISR
static float timed_from, timed_target;
static int timed_ms;               // ms since the step
static int in_band_ms;             // ms continuously inside the band
static int band_entry_ms;          // timed_ms when the beam entered the band
static int band_entry_slot;        // plot slot where that entry is drawn
static bool start_mark_pending = false;

// Most recent result. settle_report_pending hands it to the status thread
// for printing; the VGA shows it whenever have_settle_result is set.
// report_ms: >= 0 settled, -1 timed out, -2 interrupted by a new target.
#define SETTLE_TIMED_OUT (-1)
#define SETTLE_INTERRUPTED (-2)
volatile bool settle_report_pending = false;
volatile bool have_settle_result = false;
volatile float report_from, report_to;
volatile int report_ms;

// character array
char screentext[40];

// draw speed: one plot sample every PLOT_INTERVAL PWM interrupts (1 ms each),
// so the 556-sample plot spans 556 * 36 ms = 20.0 s: the whole 15 s demo
// sequence plus 5 s of settling after the final step back to horizontal
#define PLOT_INTERVAL 36
#define PLOT_SPAN_MS (PLOT_WIDTH * PLOT_INTERVAL)
#define TICK_MS 5000

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
    sprintf(screentext, "0");
    setCursor(50, 150);
    writeString(screentext);
    sprintf(screentext, "90");
    setCursor(50, 75);
    writeString(screentext);
    sprintf(screentext, "-90");
    setCursor(45, 225);
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

// Time ticks every TICK_MS under both plots. In sweep mode they count up
// from the sequence start at the left edge; in rolling mode they count back
// from "now" at the right edge.
static void drawTimeAxis(bool sweep, int count) {
    int t_ms, x;
    setTextSize(1);
    setTextColor(WHITE);
    for (t_ms = 0; t_ms <= PLOT_SPAN_MS; t_ms += TICK_MS) {
        int offset = t_ms / PLOT_INTERVAL;
        if (offset >= PLOT_WIDTH) offset = PLOT_WIDTH - 1;
        x = sweep ? LEFT_EDGE + offset : LEFT_EDGE + PLOT_WIDTH - 1 - offset;
        drawVLine(x, TOP_PLOT_BOTTOM + 1, 4, CYAN);
        drawVLine(x, BOTTOM_PLOT_BOTTOM + 1, 4, CYAN);
        if (sweep) sprintf(screentext, "%ds", t_ms / 1000);
        else       sprintf(screentext, t_ms == 0 ? "now" : "-%ds", t_ms / 1000);
        // Center-ish the label, keeping it on screen
        int label_x = x - 6;
        if (label_x > 640 - 6 * (int)strlen(screentext))
            label_x = 640 - 6 * (int)strlen(screentext);
        setCursor(label_x, BOTTOM_PLOT_BOTTOM + 6);
        writeString(screentext);
    }

    // Sequence timer, right of the motor-command title (sweep mode only)
    if (sweep) {
        sprintf(screentext, "SEQUENCE t=%.1fs",
                (double)(count * PLOT_INTERVAL) / 1000.0);
        setTextColor(YELLOW);
        setCursor(LEFT_EDGE + PLOT_WIDTH - 6 * (int)strlen(screentext), 268);
        writeString(screentext);
        setTextColor(WHITE);
    }
}

static void drawStatusText(void) {
    // Take one snapshot of each value updated by the PWM interrupt on core 0.
    float angle_snapshot = fix2float15(complementary_angle);
    float target_snapshot = requested_angle;
    float command_snapshot = fix2float15(filtered_control);
    char status_text[60];

    setTextSize(2);
    setTextColor(WHITE);
    snprintf(status_text, sizeof(status_text), "Actual: %.1f deg (%.2f rad)",
             (double)angle_snapshot, (double)(angle_snapshot * DEG_TO_RAD));
    setCursor(LEFT_EDGE, 4);
    writeString(status_text);
    snprintf(status_text, sizeof(status_text), "Target: %.1f deg (%.2f rad)",
             (double)target_snapshot, (double)(target_snapshot * DEG_TO_RAD));
    setCursor(LEFT_EDGE, 24);
    writeString(status_text);
    snprintf(status_text, sizeof(status_text), "Kp: %.2f  Ki: %.2f  Kd: %.2f",
             (double)kp, (double)ki, (double)kd);
    setCursor(LEFT_EDGE, 44);
    writeString(status_text);
    // Below the motor command plot; the top three lines are full.
    snprintf(status_text, sizeof(status_text), "Command: %d/%d (%.1f%% duty)",
             (int)lroundf(command_snapshot), WRAPVAL,
             (double)(command_snapshot * 100.0f / WRAPVAL));
    setCursor(LEFT_EDGE, 446);
    writeString(status_text);
    setTextSize(1);

    // Settling timer status, under the command line
    if (!timer_mode) {
        snprintf(status_text, sizeof(status_text), "Settle timer OFF (t to enable)");
    } else if (!have_settle_result) {
        snprintf(status_text, sizeof(status_text),
                 "Settle timer ON  band +/-%.1f deg, hold %d ms",
                 (double)SETTLE_BAND_DEG, SETTLE_HOLD_MS);
    } else if (report_ms >= 0) {
        snprintf(status_text, sizeof(status_text),
                 "Settle: %.1f -> %.1f deg in %.3f s (+/-%.1f deg)",
                 (double)report_from, (double)report_to,
                 report_ms / 1000.0, (double)SETTLE_BAND_DEG);
    } else {
        snprintf(status_text, sizeof(status_text), "Settle: %.1f -> %.1f deg %s",
                 (double)report_from, (double)report_to,
                 report_ms == SETTLE_TIMED_OUT ? "did not settle" : "interrupted");
    }
    setTextColor(MAGENTA);
    setCursor(LEFT_EDGE, 466);
    writeString(status_text);
    setTextColor(WHITE);
}

// Vertical timer markers across both plots for one plot column.
static void drawMarker(int x, int slot) {
    uint8_t mark = marker_plot[slot];
    if (mark == MARK_NONE) return;
    char color = (mark == MARK_START) ? YELLOW : MAGENTA;
    drawVLine(x, TOP_PLOT_BOTTOM - PLOT_HEIGHT, PLOT_HEIGHT + 1, color);
    drawVLine(x, BOTTOM_PLOT_BOTTOM - PLOT_HEIGHT, PLOT_HEIGHT + 1, color);
    if (mark == MARK_SETTLE) {
        char label[12];
        snprintf(label, sizeof(label), "%.2fs", marker_ms[slot] / 1000.0);
        int label_x = x + 2;
        if (label_x > 640 - 6 * (int)strlen(label))
            label_x = 640 - 6 * (int)strlen(label);
        setTextSize(1);
        setTextColor(MAGENTA);
        setCursor(label_x, TOP_PLOT_BOTTOM - PLOT_HEIGHT + 2);
        writeString(label);
        setTextColor(WHITE);
    }
}

// PWM duty cycle
volatile int control = STARTUP_OFFSET;
volatile int old_control = STARTUP_OFFSET;

volatile int throttle = 10;

// PID
// angular_rate: arm angular rate in deg/s (arm convention), straight from the gyro
float doPid(float target_angle, float current_angle, float angular_rate) {
    float error = target_angle - current_angle;
    float p_term = error * kp;
    float integral_gain = ki;

    if (integral_gain == 0.0f) {
        kerror = 0.0f;
    } else {
        float next_integral = kerror + error * CONTROL_DT;
        float next_command = STARTUP_OFFSET + p_term
                           + next_integral * integral_gain;

        // Integrate within the limits, or when error unwinds saturation.
        if ((next_command >= CONTROL_MIN && next_command <= CONTROL_MAX) ||
            (next_command > CONTROL_MAX && error < 0.0f) ||
            (next_command < CONTROL_MIN && error > 0.0f)) {
            kerror = next_integral;
        }
    }

    // Derivative: use the gyro rate (low-passed by 1/16) instead of
    // differentiating the angle, which would amplify accelerometer noise.
    // It is subtracted because a rising angle means the error is shrinking.
    static float filtered_rate = 0.0f;
    filtered_rate += (angular_rate - filtered_rate) * 0.0625f;

    return p_term + kerror * integral_gain - filtered_rate * kd;
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

// Called from the ISR: publish a settling result for serial and the VGA.
static void reportSettle(int ms) {
    report_from = timed_from;
    report_to = timed_target;
    report_ms = ms;
    have_settle_result = true;
    settle_report_pending = true;
}

// Called once per PWM interrupt (1 ms), after the angle estimate is updated.
static void updateSettleTimer(void) {
    float target = requested_angle;

    // Timer off, or motor off (button held / 'x'): nothing to time. Keep
    // tracking the target so the release step (-90 -> 0) is still detected.
    if (!timer_mode || motor_off) {
        timing = false;
        start_mark_pending = false;
        last_target = target;
        return;
    }

    if (target != last_target) {
        // New step. A step that arrives before the last one settled ends it.
        if (timing) reportSettle(SETTLE_INTERRUPTED);
        timed_from = last_target;
        timed_target = target;
        last_target = target;
        timing = true;
        timed_ms = 0;
        in_band_ms = 0;
        start_mark_pending = true;   // drawn on the next plot sample
        return;
    }
    if (!timing) return;

    timed_ms++;
    if (fabsf(fix2float15(complementary_angle) - timed_target) <= SETTLE_BAND_DEG) {
        if (in_band_ms == 0) {
            band_entry_ms = timed_ms;
            band_entry_slot = head;  // slot of the next plot sample
        }
        if (++in_band_ms >= SETTLE_HOLD_MS) {
            // Confirmed. The entry slot was written (and its marker cleared)
            // within PLOT_INTERVAL ms of entry, so it is safe to mark now.
            timing = false;
            marker_ms[band_entry_slot] = (uint16_t)band_entry_ms;
            marker_plot[band_entry_slot] = MARK_SETTLE;
            reportSettle(band_entry_ms);
        }
    } else {
        in_band_ms = 0;  // left the band: the settle point must come later
        if (timed_ms >= SETTLE_TIMEOUT_MS) {
            timing = false;
            reportSettle(SETTLE_TIMED_OUT);
        }
    }
}

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

    // Gyro angle delta (measurement times timestep) (15.16 fixed point).
    // The beam swings in the IMU's X-Z plane, i.e. it rotates about the
    // Y axis, so gyro[1] is the rate that matches the accel angle.
    // Flipped with ANGLE_DIRECTION so it matches the arm angle's sign.
    gyro_angle_delta = multfix15(gyro[1], float2fix15(CONTROL_DT)) * ANGLE_DIRECTION;

    // Complementary angle (degrees - 15.16 fixed point).
    // Added, not subtracted: for rotation about the sensor Y axis,
    // atan2(-ax, az) and gyro[1] increase together.
    complementary_angle = multfix15(complementary_angle + gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    /* ========================== PID Control ==================*/
    if (motor_off) {
        // Demo button held: motor fully off, and discard any integral built
        // up so the sequence starts without windup.
        kerror = 0.0f;
        control = 0;
    } else {
        float correction = doPid(
        requested_angle,
        fix2float15(complementary_angle),
        // Same sign as the complementary filter, which adds gyro_angle_delta
        fix2float15(gyro[1]) * ANGLE_DIRECTION
        );

        // Holding command plus the PID correction.
        float pwm_command = STARTUP_OFFSET + correction;

        // Keep the command within the valid PWM range.
        pwm_command = fminf(CONTROL_MAX, fmaxf(CONTROL_MIN, pwm_command));
        control = (int)lroundf(pwm_command);
    }
    /*========== PID Control End ==============================*/

    // Low-pass the motor command for display (1/16 per sample)
    filtered_control = filtered_control
                     + ((int2fix15(control) - filtered_control) >> 4);
    
    // Sequence just started: restart the buffer so t = 0 is the left edge,
    // and take the first sample on this interrupt.
    if (sweep_start_req) {
        sweep_start_req = false;
        head = 0;
        sweep_count = 0;
        throttle = PLOT_INTERVAL - 1;
        sweep_mode = true;
        in_band_ms = 0;  // any recorded band-entry slot is stale after the reset
    }

    updateSettleTimer();

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
        marker_plot[head] = start_mark_pending ? MARK_START : MARK_NONE;
        start_mark_pending = false;

        // Move the pointer to the oldest data
        head = ((head+1) == PLOT_WIDTH) ? 0 : (head + 1);

        // Once the sweep fills the screen, drop back to the rolling plot.
        // head has wrapped to 0, so the rolling view continues seamlessly
        // from the sweep's data.
        if (sweep_mode && ++sweep_count >= PLOT_WIDTH) {
            sweep_mode = false;
        }
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
    static int n;
    static bool sweep;

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

        // Sweep: samples 0..count-1 from the left edge (t = 0 = sequence
        // start). Rolling: the full circular buffer, oldest-->newest.
        sweep = sweep_mode;
        if (sweep) {
            n = min(sweep_count, PLOT_WIDTH);
            start = 0;
        } else {
            n = PLOT_WIDTH;
            start = head;
        }
        drawTimeAxis(sweep, n);

        for (i = 0; i < n; i++) {
            // Timer markers underneath the traces
            drawMarker(LEFT_EDGE + i, start);
            // Target first so the measured trace draws on top where they overlap
            drawPixel(LEFT_EDGE + i, plotY(TOP_PLOT_BOTTOM,
                target_plot[start], ANGLE_MIN, ANGLE_MAX), RED);
            drawPixel(LEFT_EDGE + i, plotY(TOP_PLOT_BOTTOM,
                fix2float15(angle_plot[start]), ANGLE_MIN, ANGLE_MAX), WHITE);
            drawPixel(LEFT_EDGE + i, plotY(BOTTOM_PLOT_BOTTOM,
                fix2float15(control_plot[start]), 0.0f, (float)WRAPVAL), GREEN);
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
                "command (a = angle, p/i/d = gains, x = motor off, s = start sequence, t = settle timer): ");
        serial_write_locked;
        serial_read;
        if (sscanf(pt_serial_in_buffer, " %c %c", &classifier, &extra_input) != 1) {
            sprintf(pt_serial_out_buffer, "enter a, p, i, d, x, s, or t\r\n");
            serial_write_locked;
            continue;
        }

        if (classifier == 't') {
            have_settle_result = false;
            timer_mode = !timer_mode;
            if (timer_mode) {
                snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
                         "settle timer ON: each new angle is timed until within "
                         "+/-%.1f deg for %d ms\r\n",
                         (double)SETTLE_BAND_DEG, SETTLE_HOLD_MS);
            } else {
                sprintf(pt_serial_out_buffer, "settle timer OFF\r\n");
            }
            serial_write_locked;
        } else if (classifier == 'x') {
            // TEMP: virtual button press
            press_req = true;
            sprintf(pt_serial_out_buffer, "motor off (type s to start the sequence)\r\n");
            serial_write_locked;
        } else if (classifier == 's') {
            // TEMP: virtual button release
            release_req = true;
            sprintf(pt_serial_out_buffer, "sequence started: 0 -> 30 -> -30 -> 0 deg, 5 s apart\r\n");
            serial_write_locked;
        } else if (classifier == 'p' || classifier == 'i' || classifier == 'd') {
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
            sprintf(pt_serial_out_buffer,
                    "requested angle (-90 to 90 degrees, -90 = down, 0 = horizontal): ");
            serial_write_locked;
            serial_read;
            if (sscanf(pt_serial_in_buffer, "%f %c", &angle_in, &extra_input) == 1 &&
                isfinite(angle_in) && angle_in >= -90.0f && angle_in <= 90.0f) {
                sequence_active = false;  // a typed angle overrides the demo sequence
                sweep_mode = false;       // back to the live rolling plot
                motor_off = false;        // TEMP: 'x' latches off, so a typed angle turns it back on
                requested_angle = angle_in;
                snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
                         "requested angle set to %.1f deg (%.2f rad)\r\n",
                         (double)angle_in, (double)(angle_in * DEG_TO_RAD));
            } else {
                sprintf(pt_serial_out_buffer, "invalid angle; enter a number from -90 to 90\r\n");
            }
            serial_write_locked;
        } else {
            sprintf(pt_serial_out_buffer, "unknown command; enter a, p, i, d, x, s, or t\r\n");
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
    static bool is_settle_report;
    static float from_snapshot, to_snapshot;
    static int ms_snapshot;
    next_status_us = time_us_64() + STATUS_INTERVAL_US;

    while (1) {
        // Settling results print as soon as the ISR posts them.
        PT_YIELD_UNTIL(pt, settle_report_pending || time_us_64() >= next_status_us);
        // Only the snapshots need interrupts disabled, not formatting or UART I/O.
        irq_state = save_and_disable_interrupts();
        is_settle_report = settle_report_pending;
        if (is_settle_report) {
            settle_report_pending = false;
            from_snapshot = report_from;
            to_snapshot = report_to;
            ms_snapshot = report_ms;
        } else {
            filtered_snapshot = complementary_angle;
            requested_snapshot = requested_angle;
            duty_snapshot = old_control; // Last value actually applied by the PWM ISR.
        }
        restore_interrupts(irq_state);
        if (is_settle_report && ms_snapshot >= 0) {
            snprintf(status_text, sizeof(status_text),
                     "\r\n[settle] %.1f -> %.1f deg (step %+.1f): settled in %.3f s "
                     "(band +/-%.1f deg, hold %d ms)\r\n",
                     (double)from_snapshot, (double)to_snapshot,
                     (double)(to_snapshot - from_snapshot), ms_snapshot / 1000.0,
                     (double)SETTLE_BAND_DEG, SETTLE_HOLD_MS);
        } else if (is_settle_report) {
            snprintf(status_text, sizeof(status_text),
                     "\r\n[settle] %.1f -> %.1f deg: %s\r\n",
                     (double)from_snapshot, (double)to_snapshot,
                     ms_snapshot == SETTLE_TIMED_OUT
                         ? "did not settle within the timeout"
                         : "interrupted by a new target before settling");
        } else {
            snprintf(status_text, sizeof(status_text),
                     "\r\nfiltered=%.1f deg (%.2f rad)  requested=%.1f deg (%.2f rad)  duty=%d/%d\r\n",
                     (double)fix2float15(filtered_snapshot),
                     (double)(fix2float15(filtered_snapshot) * DEG_TO_RAD),
                     (double)requested_snapshot, (double)(requested_snapshot * DEG_TO_RAD),
                     duty_snapshot, WRAPVAL);
        }

        PT_YIELD_UNTIL(pt, !serial_tx_busy);
        serial_tx_busy = true;
        for (send_index = 0; status_text[send_index] != '\0'; ++send_index) {
            PT_YIELD_UNTIL(pt, uart_is_writable(UART_ID));
            uart_putc(UART_ID, status_text[send_index]);
        }
        serial_tx_busy = false;
        if (!is_settle_report) {
            next_status_us += STATUS_INTERVAL_US;
            if (next_status_us <= time_us_64()) {
                next_status_us = time_us_64() + STATUS_INTERVAL_US;
            }
        }
    }
    PT_END(pt);
}

// Demo button thread: polls BUTTON_PIN every 10 ms with a simple debounce.
// While the button is held the motor is off (beam hangs down). On release
// (time = 0) the target steps through horizontal, 30 deg above, 30 deg
// below, and back to horizontal at 0, 5, 10 and 15 s, using whatever PID
// gains are currently set. Millisecond-scale timing is fine here, so this
// runs as a protothread rather than in the ISR.
static PT_THREAD(protothread_button(struct pt *pt)) {
    PT_BEGIN(pt);
    static bool stable_pressed = false; // debounced button state
    static bool last_raw = false;       // previous raw read
    static int same_count = 0;          // consecutive identical raw reads
    static uint64_t seq_start_us;       // time of button release
    static int seq_step;                // next step to apply (1..3)
    static uint64_t now;
    static bool raw;

    while (1) {
        PT_YIELD_usec(BUTTON_POLL_US);
        now = time_us_64();

        // Debounce: accept a new state only after it reads the same
        // BUTTON_STABLE_READS times in a row (~20 ms).
        raw = !gpio_get(BUTTON_PIN);  // active low
        same_count = (raw == last_raw) ? same_count + 1 : 0;
        last_raw = raw;

        if (same_count >= BUTTON_STABLE_READS && raw != stable_pressed) {
            stable_pressed = raw;
            if (stable_pressed) press_req = true;
            else                release_req = true;
        }

        // Physical button edges and TEMP serial 'x'/'s' commands both land here.
        if (press_req || release_req) {
            if (press_req) {
                press_req = false;
                release_req = false;
                // Pressed: motor off, cancel any sequence in progress.
                sequence_active = false;
                requested_angle = SEQ_DOWN;
                motor_off = true;
                sweep_mode = false;   // live rolling plot while held
            } else {
                // Released: time = 0, target horizontal, motor back on.
                release_req = false;
                // motor_off goes false BEFORE the target changes, so the
                // settle timer (which ignores steps while the motor is off)
                // sees the -90 -> 0 release step.
                motor_off = false;
                requested_angle = SEQ_HORIZONTAL;
                seq_start_us = now;
                seq_step = 1;
                sequence_active = true;
                sweep_start_req = true;  // plot t = 0 at the left edge
            }
        }

        // Step the target every SEQ_STEP_US after release.
        if (sequence_active && now - seq_start_us >= seq_step * SEQ_STEP_US) {
            if (seq_step == 1) {
                requested_angle = SEQ_ABOVE;       // t = 5 s
            } else if (seq_step == 2) {
                requested_angle = SEQ_BELOW;       // t = 10 s
            } else {
                requested_angle = SEQ_HORIZONTAL;  // t = 15 s, sequence done
                sequence_active = false;
            }
            seq_step++;
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

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    // Initialize accelerometer filters
    filtered_ax = acceleration[0];
    filtered_az = acceleration[2];

    // Get initial absolute angle from accelerometer
    accel_angle = armAngleFromAccel(filtered_ax, filtered_az);

    // Start complementary filter at accelerometer estimate
    complementary_angle = accel_angle;

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
    pwm_set_chan_level(slice_num, PWM_CHAN_A, STARTUP_OFFSET);

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

    // Demo button: input with pull-up, pressed reads low
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_button);
    pt_add_thread(protothread_status);
    pt_schedule_start;
}
