#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

// OPA323 output saturates ~34 mV below the 3.3 V supply rail.
// Measured empirically; update if supply voltage changes.
const float SENSOR_SAT_V = 3.2;

adsGain_t gains[] = {
  GAIN_TWOTHIRDS,  // 0: ±6.144V
  GAIN_ONE,        // 1: ±4.096V
  GAIN_TWO,        // 2: ±2.048V
  GAIN_FOUR,       // 3: ±1.024V
  GAIN_EIGHT,      // 4: ±0.512V
  GAIN_SIXTEEN     // 5: ±0.256V
};
float gainVoltages[] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};
int currentGain = 1;

void setup() {
  Serial.begin(115200);
  ads.begin();
  ads.setGain(gains[currentGain]);
  ads.setDataRate(RATE_ADS1115_860SPS);
  // NOTE: 400kHz I2C caused comms failure (all zeros) with the Soldered
  // breakout's onboard pull-ups over jumper wires. Leave at default 100kHz.
  // Wire.setClock(400000);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'r') {
      int16_t raw = ads.readADC_SingleEnded(0);

      // ADC saturation: raw hit the top of the signed 16-bit range.
      bool adcSat = (raw >= 32767);

      // Sensor saturation: op-amp output near the supply rail.
      // Only possible when the gain full-scale exceeds SENSOR_SAT_V;
      // otherwise the ADC overflows before the sensor can saturate.
      bool sensorSat = false;
      if (gainVoltages[currentGain] > SENSOR_SAT_V) {
        int16_t satThreshold = (int16_t)(SENSOR_SAT_V / gainVoltages[currentGain] * 32767);
        sensorSat = (raw >= satThreshold);
      }

      Serial.print(raw);
      Serial.print(",");
      Serial.print(sensorSat ? 1 : 0);
      Serial.print(",");
      Serial.println(adcSat ? 1 : 0);

    } else if (cmd == 'g') {
      while (!Serial.available());
      char c = Serial.read();
      int g = c - '0';
      if (g >= 0 && g <= 5) {
        currentGain = g;
        ads.setGain(gains[currentGain]);
        Serial.println("ok");
      } else {
        Serial.println("err");
      }

    } else if (cmd == 'G') {
      Serial.println(currentGain);
    }
  }
}
