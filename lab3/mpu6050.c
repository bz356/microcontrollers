/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 *
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

// Free a bus left mid-transfer by a Pico reset: clock out 9 bits (the IMU sees
// a NACK and lets go of SDA), then send a STOP. Lines are only pulled low or
// released, never driven high. Call before i2c_init().
void mpu6050_bus_recover(void) {
    gpio_init(SDA_PIN);
    gpio_init(SCL_PIN);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    // Output latch stays 0; switching direction picks "pull low" or "release"
    gpio_put(SDA_PIN, 0);
    gpio_put(SCL_PIN, 0);
    gpio_set_dir(SDA_PIN, GPIO_IN);
    gpio_set_dir(SCL_PIN, GPIO_IN);
    sleep_us(5);

    // 9 clocks at ~100 kHz
    for (int i = 0; i < 9; i++) {
        gpio_set_dir(SCL_PIN, GPIO_OUT);  // SCL low
        sleep_us(5);
        gpio_set_dir(SCL_PIN, GPIO_IN);   // SCL released high
        sleep_us(5);
    }

    // STOP: SDA goes low -> high while SCL is high
    gpio_set_dir(SCL_PIN, GPIO_OUT); sleep_us(5);  // SCL low
    gpio_set_dir(SDA_PIN, GPIO_OUT); sleep_us(5);  // SDA low
    gpio_set_dir(SCL_PIN, GPIO_IN);  sleep_us(5);  // SCL high
    gpio_set_dir(SDA_PIN, GPIO_IN);  sleep_us(5);  // SDA high = STOP
}

// Returns false if the IMU does not acknowledge the wake-up write. All setup
// writes are still sent. Timeouts keep a stuck bus from hanging startup.
bool mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in
    // different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    bool acked = i2c_write_timeout_us(I2C_CHAN, ADDRESS, buf, 2, false, 1000) == 2;

    // Set gyro sample rate (set to 1KHz, same as accel)
    uint8_t gyro_rate[] = {0x19, 0b00000111} ;
    i2c_write_timeout_us(I2C_CHAN, ADDRESS, gyro_rate, 2, false, 1000);

    // Configure the Gyro range (+/- 250 deg/s)
    uint8_t gyro_settings[] = {0x1b, 0b00000000} ;
    i2c_write_timeout_us(I2C_CHAN, ADDRESS, gyro_settings, 2, false, 1000);

    // Configure the Accel range (+/- 2g's)
    uint8_t accel_settings[] = {0x1c, 0b00000000} ;
    i2c_write_timeout_us(I2C_CHAN, ADDRESS, accel_settings, 2, false, 1000);

    // Configure interrupt pin
    uint8_t pin_settings[] = {0x37, 0b00010000} ;
    i2c_write_timeout_us(I2C_CHAN, ADDRESS, pin_settings, 2, false, 1000);

    // Configure data ready interrupt
    uint8_t int_config[] = {0x38, 0x01} ;
    i2c_write_timeout_us(I2C_CHAN, ADDRESS, int_config, 2, false, 1000);
    return acked;
}

// Release the I2C peripheral, unstick the bus, and set the I2C peripheral up
// again. Used at boot and whenever reads keep failing at runtime (an SDK
// timeout can leave the controller or the IMU mid-transfer). Call
// mpu6050_reset() afterwards to reconfigure the IMU.
void mpu6050_bus_reinit(void) {
    i2c_deinit(I2C_CHAN);
    mpu6050_bus_recover();
    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    // Internal pull-ups (~50k) are weak for 400 kHz; external pull-ups on
    // the breakout board do the real work.
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// Reads accel and gyro in one 14-byte burst (0x3B..0x48: accel XYZ, temp,
// gyro XYZ), so all six values come from the same sample. Returns false if
// any part of the transfer fails, and then leaves accel[] and gyro[]
// untouched so the caller can keep the last good values.
bool mpu6050_read_raw(fix15 accel[3], fix15 gyro[3]) {
    uint8_t buffer[14];
    uint8_t reg = 0x3B;

    // Send the start register, keeping control of the bus (repeated start).
    // 2 bytes at 400 kHz take ~50 us.
    if (i2c_write_timeout_us(I2C_CHAN, ADDRESS, &reg, 1, true, 200) != 1) {
        return false;
    }
    // 15 bytes (address + 14 data) at 400 kHz take ~340 us.
    if (i2c_read_timeout_us(I2C_CHAN, ADDRESS, buffer, 14, false, 600) != 14) {
        return false;
    }

    for (int i = 0; i < 3; i++) {
        int16_t temp_accel = (int16_t)(buffer[i<<1] << 8 | buffer[(i<<1) + 1]);
        accel[i] = (fix15)temp_accel << 2;  // +/-2 g range: 16384 LSB/g -> g's (fixed point)

        // Gyro starts at byte 8 (bytes 6-7 are temperature)
        int16_t temp_gyro = (int16_t)(buffer[8 + (i<<1)] << 8 | buffer[8 + (i<<1) + 1]);
        gyro[i] = multfix15((fix15)temp_gyro, 500<<16);  // +/-250 deg/s range: 131 LSB/(deg/s) -> deg/s
    }
    return true;
}
/////////////////////////////////////////////////////////////////