/*  ALSMD.ino
 *  This .ino file is a simple usage example of the ALSMD.h header
 */

#include "ALSMD.h"

LSM303 sensor(1, 1);
fvec3 magData;
fvec3 accelData;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    sensor.configure(ACCEL_RATE_10HZ, MAG_RATE_0_75HZ, ACCEL_MODE_LOW_POWER, MAG_MODE_SINGLE,
                     ACCEL_SCALE_2G, MAG_SCALE_8_1);
    if (!sensor.init()) {
        Serial.println("Failed to initialize LSM303!");
        while (1) {
        }
    }
    Serial.println("LSM303 initialized successfully");

    sensor.statusDump();
}

void loop() {
    sleep(1);

    sensor.readMag(magData);
    sensor.readAccel(accelData);

    Serial.print("aX = ");
    Serial.print(accelData.x);
    Serial.print(", aY = ");
    Serial.print(accelData.y);
    Serial.print(", aZ = ");
    Serial.println(accelData.z);
    Serial.print("mX = ");
    Serial.print(magData.x);
    Serial.print(", mY = ");
    Serial.print(magData.y);
    Serial.print(", mZ = ");
    Serial.println(magData.z);
}
