#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

void setup() {
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensorCount);
  delay(500);

  for(uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  Serial.println(qtr.calibrationOn.maximum[0]);
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  /*for(int i = 0; i < sensorCount; i++) {
    Serial.print(sensorValues[i] > 700 ? 1 : 0);
  }
  Serial.println();*/
  Serial.println(position);
}
