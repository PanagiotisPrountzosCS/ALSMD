#ifndef _ALSMD_H
#define _ALSMD_H

#include <Wire.h>

typedef uint8_t byte;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} i16vec3;

typedef struct {
    float x;
    float y;
    float z;
} fvec3;

// LSM303 Accelerometer Registers
enum AccelRegisters {
    CTRL_REG1_A = 0x20,      // Control register 1
    CTRL_REG2_A = 0x21,      // Control register 2
    CTRL_REG3_A = 0x22,      // Control register 3
    CTRL_REG4_A = 0x23,      // Control register 4
    CTRL_REG5_A = 0x24,      // Control register 5
    CTRL_REG6_A = 0x25,      // Control register 6
    REFERENCE_A = 0x26,      // Reference/datacapture
    STATUS_REG_A = 0x27,     // Status register
    OUT_X_L_A = 0x28,        // X-axis acceleration data low byte
    OUT_X_H_A = 0x29,        // X-axis acceleration data high byte
    OUT_Y_L_A = 0x2A,        // Y-axis acceleration data low byte
    OUT_Y_H_A = 0x2B,        // Y-axis acceleration data high byte
    OUT_Z_L_A = 0x2C,        // Z-axis acceleration data low byte
    OUT_Z_H_A = 0x2D,        // Z-axis acceleration data high byte
    FIFO_CTRL_REG_A = 0x2E,  // FIFO control register
    FIFO_SRC_REG_A = 0x2F,   // FIFO source register
    INT1_CFG_A = 0x30,       // Interrupt 1 configuration
    INT1_SRC_A = 0x31,       // Interrupt 1 source
    INT1_THS_A = 0x32,       // Interrupt 1 threshold
    INT1_DURATION_A = 0x33,  // Interrupt 1 duration
    INT2_CFG_A = 0x34,       // Interrupt 2 configuration
    INT2_SRC_A = 0x35,       // Interrupt 2 source
    INT2_THS_A = 0x36,       // Interrupt 2 threshold
    INT2_DURATION_A = 0x37,  // Interrupt 2 duration
    CLICK_CFG_A = 0x38,      // Click configuration
    CLICK_SRC_A = 0x39,      // Click source
    CLICK_THS_A = 0x3A,      // Click threshold
    TIME_LIMIT_A = 0x3B,     // Time limit
    TIME_LATENCY_A = 0x3C,   // Time latency
    TIME_WINDOW_A = 0x3D     // Time window
};

// LSM303 Magnetometer Registers
enum MagRegisters {
    CRA_REG_M = 0x00,     // Configuration register A
    CRB_REG_M = 0x01,     // Configuration register B
    MR_REG_M = 0x02,      // Mode register
    OUT_X_H_M = 0x03,     // X-axis magnetic data high byte
    OUT_X_L_M = 0x04,     // X-axis magnetic data low byte
    OUT_Z_H_M = 0x05,     // Z-axis magnetic data high byte (Note: Z before Y in register map)
    OUT_Z_L_M = 0x06,     // Z-axis magnetic data low byte
    OUT_Y_H_M = 0x07,     // Y-axis magnetic data high byte
    OUT_Y_L_M = 0x08,     // Y-axis magnetic data low byte
    SR_REG_M = 0x09,      // Status register
    IRA_REG_M = 0x0A,     // Identification register A
    IRB_REG_M = 0x0B,     // Identification register B
    IRC_REG_M = 0x0C,     // Identification register C
    TEMP_OUT_H_M = 0x31,  // Temperature data high byte (DLHC/DLM only)
    TEMP_OUT_L_M = 0x32   // Temperature data low byte (DLHC/DLM only)
};

// Constants for configuration
enum AccelScale {
    ACCEL_SCALE_2G = 0,
    ACCEL_SCALE_4G,
    ACCEL_SCALE_8G,
    ACCEL_SCALE_16G,
    ACCEL_SCALE_COUNT
};

enum MagScale {
    // this enum needs to start from 1 because the GN bits start from 001 for a gain of 1.3 :(
    MAG_SCALE_1_3 = 1,  // ±1.3 gauss
    MAG_SCALE_1_9,      // ±1.9 gauss
    MAG_SCALE_2_5,      // ±2.5 gauss
    MAG_SCALE_4_0,      // ±4.0 gauss
    MAG_SCALE_4_7,      // ±4.7 gauss
    MAG_SCALE_5_6,      // ±5.6 gauss
    MAG_SCALE_8_1       // ±8.1 gauss
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

enum AccelSensitivity {
    ACCEL_SENSITIVITY_2G = 1,
    ACCEL_SENSITIVITY_4G = 2,
    ACCEL_SENSITIVITY_8G = 4,
    ACCEL_SENSITIVITY_16G = 12
};

enum MagSensitivityXY {
    MAG_SENSITIVITY_XY_1_3 = 1100,
    MAG_SENSITIVITY_XY_1_9 = 855,
    MAG_SENSITIVITY_XY_2_5 = 670,
    MAG_SENSITIVITY_XY_4_0 = 450,
    MAG_SENSITIVITY_XY_4_7 = 400,
    MAG_SENSITIVITY_XY_5_6 = 330,
    MAG_SENSITIVITY_XY_8_1 = 230
};

enum MagSensitivityZ {
    MAG_SENSITIVITY_Z_1_3 = 980,
    MAG_SENSITIVITY_Z_1_9 = 760,
    MAG_SENSITIVITY_Z_2_5 = 600,
    MAG_SENSITIVITY_Z_4_0 = 400,
    MAG_SENSITIVITY_Z_4_7 = 355,
    MAG_SENSITIVITY_Z_5_6 = 295,
    MAG_SENSITIVITY_Z_8_1 = 205
};

class LSM303 {
public:
    LSM303(bool magEnable, bool accelEnable, uint8_t accelAddr = 0x19, uint8_t magAddr = 0x1E) {
        _magEnable = magEnable;
        _accelEnable = accelEnable;
        _accelAddress = accelAddr;
        _magAddress = magAddr;
    }  // Default I2C addresses

    bool checkI2CCommunication() {
        // just confirm we can begin and end transmission to the i2c slaves
        Wire.beginTransmission(_accelAddress);
        byte accelStatus = Wire.endTransmission();
        if (accelStatus != 0) {
            Serial.print("Accel fail. Error code : ");
            Serial.println(accelStatus);
            return false;
        }

        Wire.beginTransmission(_magAddress);
        byte magStatus = Wire.endTransmission();
        if (magStatus != 0) {
            Serial.print("Mag fail. Error code : ");
            Serial.println(magStatus);
            return false;
        }

        return true;
    }

    bool defaultInit() {
        // confirm I2C communication
        if (!checkI2CCommunication()) return false;

        if (_accelEnable) {
            // initialize the accelerometer's and the magnetometer's control registers
            // 0x57 in register 0x20 means 100Hz normal power mode and xyz enabled
            writeByte(_accelAddress, CTRL_REG1_A, 0x57);

            // high resolution disabled + 2g accelerometer range
            _accelScale = ACCEL_SCALE_2G;
            writeByte(_accelAddress, CTRL_REG4_A, _accelScale << 4);
        }

        if (_magEnable) {
            // this controls the rate at which data is written in the output registers
            writeByte(_magAddress, CRA_REG_M, 0x10);

            _magScale = MAG_SCALE_8_1;
            writeByte(_magAddress, CRB_REG_M, _magScale << 5);

            // set magnetometer in continuous conversion mode
            writeByte(_magAddress, MR_REG_M, 0x00);
        }
        return true;
    }

    void enableMag() { _magEnable = true; }

    void enableAccel() { _accelEnable = true; }

    void disableMag() { _magEnable = false; }

    void disableAccel() { _accelEnable = false; }

    void readRawAccel(i16vec3& vec) {
        if (!_accelEnable) return;
        uint8_t buffer[6];
        // each value is 2 bytes, so we stack allocate 6 as a buffer

        // read all 6
        readByteArray(_accelAddress, OUT_X_L_A, buffer, 6);

        vec.x = (int16_t)((buffer[1] << 8) | buffer[0]) >> 4;
        vec.y = (int16_t)((buffer[3] << 8) | buffer[2]) >> 4;
        vec.z = (int16_t)((buffer[5] << 8) | buffer[4]) >> 4;
        // right bit shift might be variable
    }

    void readRawMag(i16vec3& vec) {
        if (!_magEnable) return;
        uint8_t buffer[6];

        readByteArray(_magAddress, OUT_X_L_M, buffer, 6);

        vec.x = (int16_t)((buffer[1] << 8) | buffer[0]);
        vec.y = (int16_t)((buffer[5] << 8) | buffer[4]);
        vec.z = (int16_t)((buffer[3] << 8) | buffer[2]);
    }

    void readAccel(fvec3& vec) {
        i16vec3 temp;
        readRawAccel(temp);
        processAccelData(vec, temp);
    }

    void readMag(fvec3& vec) {
        i16vec3 temp;
        readRawMag(temp);
        processMagData(vec, temp);
    }

    void processAccelData(fvec3& out, const i16vec3 in) {
        // sensitivity switch
        switch (_accelScale) {
            case ACCEL_SCALE_2G:
                out.x = in.x;
                out.y = in.y;
                out.z = in.z;
                break;
            case ACCEL_SCALE_4G:
                out.x = ACCEL_SENSITIVITY_4G * in.x;
                out.y = ACCEL_SENSITIVITY_4G * in.y;
                out.z = ACCEL_SENSITIVITY_4G * in.z;
                break;
            case ACCEL_SCALE_8G:
                out.x = ACCEL_SENSITIVITY_8G * in.x;
                out.y = ACCEL_SENSITIVITY_8G * in.y;
                out.z = ACCEL_SENSITIVITY_8G * in.z;
                break;
            case ACCEL_SCALE_16G:
                out.x = ACCEL_SENSITIVITY_16G * in.x;
                out.y = ACCEL_SENSITIVITY_16G * in.y;
                out.z = ACCEL_SENSITIVITY_16G * in.z;
                break;
            default:
                out.x = 0;
                out.y = 0;
                out.z = 0;
        }
        // mg to g
        out.x /= 1000;
        out.y /= 1000;
        out.z /= 1000;
    }

    void processMagData(fvec3& out, const i16vec3 in) {
        switch (_magScale) {
            case MAG_SCALE_1_3:
                out.x = (in.x / (float)MAG_SENSITIVITY_XY_1_3) * 100.0f;
                out.y = (in.y / (float)MAG_SENSITIVITY_XY_1_3) * 100.0f;
                out.z = (in.z / (float)MAG_SENSITIVITY_Z_1_3) * 100.0f;
                break;

            case MAG_SCALE_1_9:
                out.x = (in.x / (float)MAG_SENSITIVITY_XY_1_9) * 100.0f;
                out.y = (in.y / (float)MAG_SENSITIVITY_XY_1_9) * 100.0f;
                out.z = (in.z / (float)MAG_SENSITIVITY_Z_1_9) * 100.0f;
                break;

            case MAG_SCALE_2_5:
                out.x = (in.x / (float)MAG_SENSITIVITY_XY_2_5) * 100.0f;
                out.y = (in.y / (float)MAG_SENSITIVITY_XY_2_5) * 100.0f;
                out.z = (in.z / (float)MAG_SENSITIVITY_Z_2_5) * 100.0f;
                break;

            case MAG_SCALE_4_0:
                out.x = (in.x / (float)MAG_SENSITIVITY_XY_4_0) * 100.0f;
                out.y = (in.y / (float)MAG_SENSITIVITY_XY_4_0) * 100.0f;
                out.z = (in.z / (float)MAG_SENSITIVITY_Z_4_0) * 100.0f;
                break;

            case MAG_SCALE_4_7:
                out.x = (in.x / (float)MAG_SENSITIVITY_XY_4_7) * 100.0f;
                out.y = (in.y / (float)MAG_SENSITIVITY_XY_4_7) * 100.0f;
                out.z = (in.z / (float)MAG_SENSITIVITY_Z_4_7) * 100.0f;
                break;

            case MAG_SCALE_5_6:
                out.x = (in.x / (float)MAG_SENSITIVITY_XY_5_6) * 100.0f;
                out.y = (in.y / (float)MAG_SENSITIVITY_XY_5_6) * 100.0f;
                out.z = (in.z / (float)MAG_SENSITIVITY_Z_5_6) * 100.0f;
                break;

            case MAG_SCALE_8_1:
                out.x = (in.x / (float)MAG_SENSITIVITY_XY_8_1) * 100.0f;
                out.y = (in.y / (float)MAG_SENSITIVITY_XY_8_1) * 100.0f;
                out.z = (in.z / (float)MAG_SENSITIVITY_Z_8_1) * 100.0f;
                break;

            default:
                out.x = 0;
                out.y = 0;
                out.z = 0;
                break;
        }
    }

    void setAccelScale(uint8_t newScale) {
        if (newScale != _accelScale && newScale < ACCEL_SCALE_COUNT)
            _accelScale = newScale;
        else
            return;
        // Read current register value
        uint8_t current = readByte(_accelAddress, CTRL_REG4_A);

        // Keep other control flags
        current &= 0b11001111;

        // Set the new scale bits
        current |= (newScale << 4);

        // Rewrite control register 4
        writeByte(_accelAddress, CTRL_REG4_A, current);
    }

    void setMagScale(uint8_t newScale) {
        if (newScale != _magScale && newScale <= 7)
            _magScale = newScale;
        else
            return;

        // Write the gain setting to CRB_REG_M
        // The gain bits are in bits 5-7
        writeByte(_magAddress, CRB_REG_M, newScale << 5);
    }

private:
    void writeByte(byte dev_addr, byte reg_addr, byte data) {
        Wire.beginTransmission(dev_addr);
        Wire.write(reg_addr);
        Wire.write(data);
        Wire.endTransmission();
    }

    byte readByte(byte dev_addr, byte reg_addr) {
        Wire.beginTransmission(dev_addr);
        Wire.write(reg_addr);
        Wire.endTransmission(false);

        Wire.requestFrom(dev_addr, 1);
        return Wire.read();
    }

    void readByteArray(byte dev_addr, byte reg_addr, byte* buf, byte count) {
        Wire.beginTransmission(dev_addr);
        Wire.write(reg_addr | 0x80);  // read register auto increment flag
        Wire.endTransmission(false);

        Wire.requestFrom(dev_addr, count);
        for (uint8_t i = 0; i < count && Wire.available(); i++) {
            buf[i] = Wire.read();
        }
    }
    // I2C addresses
    uint8_t _accelAddress;
    uint8_t _magAddress;

    // Configuration state
    uint8_t _accelScale;
    uint8_t _magScale;
    uint8_t _accelRate;
    uint8_t _magRate;

    // Module states
    bool _magEnable;
    bool _accelEnable;
};

#endif