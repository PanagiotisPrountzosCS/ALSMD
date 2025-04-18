/*  ALSMD.ino
 *  This .ino file is a simple usage example of the ALSMD.h header
 */

#include "ALSMD.h"

LSM303 sensor;
int16_t aX;
int16_t aY;
int16_t aZ;
int16_t mX;
int16_t mY;
int16_t mZ;

void setup() {
    if (!sensor.init()) {
        Serial.println("Failed to initialize LSM303!");
        while (1) {
        }
        Serial.println("LSM303 initialized successfully");
    }
}

void loop() {
    sleep(500);

    sensor.readAccel(&aX, &aY, &aZ);
    sensor.readAccel(&mX, &mY, &mZ);

    Serial.print("aX = ");
    Serial.println(aX);
    Serial.print("aY = ");
    Serial.println(aY);
    Serial.print("aZ = ");
    Serial.println(aZ);
    Serial.print("mX = ");
    Serial.println(mX);
    Serial.print("mY = ");
    Serial.println(mY);
    Serial.print("mZ = ");
    Serial.println(mZ);
}
