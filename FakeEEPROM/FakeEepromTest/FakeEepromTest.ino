#include <Wire.h>

#define DeviceAddr 0b1010000

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  Wire.begin();
}

uint8_t WordAddr = 0;
void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(DeviceAddr);
  Wire.write(WordAddr);
  Wire.endTransmission();
  Wire.requestFrom(DeviceAddr, 1);
  uint8_t x = Wire.read();
  Serial.print(WordAddr); Serial.print(": ");
  Serial.println(x);
  WordAddr++;
  if (WordAddr > 31) {
    WordAddr = 0;
    delay(500);
  }
}
