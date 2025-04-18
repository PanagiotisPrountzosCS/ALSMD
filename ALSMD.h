#ifndef _ALSMD_H
#define _ALSMD_H

#include <Wire.h>

// Constants for configuration
enum AccelRange {
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G,
    ACCEL_RANGE_COUNT
};

enum MagGain {
    MAG_GAIN_1_3 = 0,  // ±1.3 gauss
    MAG_GAIN_1_9,      // ±1.9 gauss
    MAG_GAIN_2_5,      // ±2.5 gauss
    MAG_GAIN_4_0,      // ±4.0 gauss
    MAG_GAIN_4_7,      // ±4.7 gauss
    MAG_GAIN_5_6,      // ±5.6 gauss
    MAG_GAIN_8_1,      // ±8.1 gauss
    MAG_GAIN_COUNT
};

enum DataRate {
    RATE_0_75HZ = 0,
    RATE_1_5HZ,
    RATE_3HZ,
    RATE_7_5HZ,
    RATE_15HZ,
    RATE_30HZ,
    RATE_75HZ,
    RATE_220HZ,
    RATE_COUNT
};

class LSM303 {
public:
    LSM303(uint8_t accelAddr = 0x19, uint8_t magAddr = 0x1E);  // Default I2C addresses

    bool init();

    bool setAccelRange(uint8_t range);
    bool setMagGain(uint8_t gain);
    bool setAccelDataRate(uint8_t rate);
    bool setMagDataRate(uint8_t rate);

    bool enableAccel(bool enable = true);
    bool enableMag(bool enable = true);
    bool sleep();
    bool wake();

    int16_t readAccelX();
    int16_t readAccelY();
    int16_t readAccelZ();
    int16_t readMagX();
    int16_t readMagY();
    int16_t readMagZ();

    bool readAccel(int16_t* x, int16_t* y, int16_t* z);
    bool readMag(int16_t* x, int16_t* y, int16_t* z);

    void calibrateAccel();
    void calibrateMag();
    void setAccelBias(int16_t x, int16_t y, int16_t z);
    void setMagBias(int16_t x, int16_t y, int16_t z);
    void setMagScale(float x, float y, float z);

    float getHeading();
    float getTiltCompensatedHeading();
    float getTemperature();

private:
    // I2C addresses (may vary by exact chip variant)
    uint8_t _accelAddress;
    uint8_t _magAddress;

    // Configuration state
    uint8_t _accelScale;  // Current accelerometer range setting
    uint8_t _magGain;     // Current magnetometer gain setting
    uint8_t _accelRate;   // Current accelerometer sampling rate
    uint8_t _magRate;     // Current magnetometer sampling rate
    bool _accelEnabled;   // Is accelerometer currently enabled?
    bool _magEnabled;     // Is magnetometer currently enabled?

    // Calibration values
    int16_t _accelBias[3];  // Offsets for x, y, z
    int16_t _magBias[3];    // Offsets for x, y, z
    float _magScale[3];     // Scale factors for x, y, z

    // Helper methods for I2C communication
    bool writeAccelReg(uint8_t reg, uint8_t value);
    bool writeMagReg(uint8_t reg, uint8_t value);
    uint8_t readAccelReg(uint8_t reg);
    uint8_t readMagReg(uint8_t reg);
    bool readAccelRegs(uint8_t reg, uint8_t* buffer, uint8_t len);
    bool readMagRegs(uint8_t reg, uint8_t* buffer, uint8_t len);
};

#endif _ALSMD_H