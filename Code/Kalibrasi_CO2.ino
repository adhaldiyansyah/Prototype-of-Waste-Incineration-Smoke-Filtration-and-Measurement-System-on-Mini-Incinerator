#include <Wire.h>

// Kalibrasi CO2
#define waktukalibrasi 50  
#define interval 1    
#define sampel (waktukalibrasi / interval)

const int analogPins_CO2[3] = {A3, A4, A5}; 
const float Vcc = 5.0;
const float RL = 10000;

float Ro_CO2[3] = {0, 0, 0};  

// menghitung resistansi sensor dari nilai ADC
float getSensorResistance(int sensorValue) {
  float v = (sensorValue / 1023.0) * Vcc;
  return ((Vcc - v) / v) * RL;
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Mulai Kalibrasi Sensor CO2...");
  
  for (int sample = 0; sample < sampel; sample++) {
    for (int i = 0; i < 3; i++) {
      int sensorValue = analogRead(analogPins_CO2[i]);
      float Rs = getSensorResistance(sensorValue);
      Ro_CO2[i] += Rs / sampel;
    }

    // Tampilkan waktu tersisa di Serial
    int remainingTime = waktukalibrasi - (sample * interval);
    int minutes = remainingTime / 60;
    int seconds = remainingTime % 60;
    Serial.print("Waktu tersisa: ");
    Serial.print(minutes);
    Serial.print("m ");
    Serial.print(seconds);
    Serial.println("s");

    delay(interval * 1000);
  }

  Serial.println("Kalibrasi Selesai.");
  for (int i = 0; i < 3; i++) {
    Serial.print("Ro_CO2 Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(Ro_CO2[i], 2);
  }
}

void loop() {
  
}
