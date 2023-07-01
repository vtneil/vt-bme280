#include <Arduino.h>
#include "vt_bme280"

using namespace vt;

bme280_t sensor;

void setup() {
    Serial.begin(115200);

    delay(1000);
    Serial.println(F("Serial OK!"));
    Serial.println(sensor.begin());
}

void loop() {
    float temp = sensor.read_temperature_c();
    float hum = sensor.read_humidity();
    float pres = sensor.read_pressure();

    Serial.println(temp);
    Serial.println(hum);
    Serial.println(pres);
    Serial.println("--- --- ---");

    delay(1000);
}
