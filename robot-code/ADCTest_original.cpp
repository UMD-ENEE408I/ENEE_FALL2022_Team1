#include <Arduino.h>
#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

void setup() {
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  Serial.begin(115200);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  
}

void loop() {
  int adc1_buf[8];
  int adc2_buf[8];

  int t_start = micros();
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }
  int t_end = micros();

  for (int i = 0; i < 8; i++) {
    Serial.print(adc1_buf[i]); Serial.print("\t");
    Serial.print(adc2_buf[i]); Serial.print("\t");
  }

  Serial.print(t_end - t_start);
  Serial.println();

  delay(100);
}
