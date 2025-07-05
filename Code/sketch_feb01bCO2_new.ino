#include <Wire.h>
#include <RTClib.h>
#include <SdFat.h>

SdFat SD;
File dataFile;
RTC_DS3231 rtc;


//Kalibrasi CO2
#define CALIBRATION_TIME 50  // 1 jam dalam detik
#define SAMPLE_INTERVAL 1      // Interval pengambilan sampel dalam detik
#define SAMPLES (CALIBRATION_TIME / SAMPLE_INTERVAL)

unsigned long previousMillis = 0;
const long interval = 1000;  // Interval 1 detik untuk membaca sensor


//int flamesensor = 3;
//int relayA = 5;
bool flamestate = false;
bool lastRelayState = HIGH; // Simpan status relay sebelumnya (HIGH = Mati, LOW = Aktif)
bool hasWrittenOffStatus = false; // Flag untuk menandai apakah status "Mati" sudah ditulis

//bool srelay = false;

// Konfigurasi pin dan variabel sensor
const uint8_t chipSelect = 10;
int LED_HIJAU = 7;
int LED_MERAH = 8;
int LED_KUNING = 6;
int flamesensor = 3;
int relayA = 5;
const int analogPins_CO[3] = {A0, A1, A2};
const int analogPins_CO2[3] = {A3, A4, A8};
const int measurePins[3] = {A12, A13, A14};
const int ledPowers[3] = {38, 39, 40};
float Rs_CO2[3];
float Ro_CO2[3]= {68309.04, 35235.45, 55683.28};//{78309.04, 11331.17, 86683.28}; //{251258.54, 248243.45, 224216.57}; Defautl //{74047.22, 31813.89, 31227.57}; Kalibrasi kedua
float ratio_CO2[3];
float ppmCO2[3];
float Vout_CO[3];
float ratio_CO[3];
float ppmCO[3];
const float Vcc = 5.0;
const float RL = 10000;
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

// Array nama bulan untuk format nama file
const char* namaBulan[] = {
  "", "Januari", "Februari", "Maret", "April", "Mei", "Juni",
  "Juli", "Agustus", "September", "Oktober", "November", "Desember"
};

//Parameter untuk SD card
bool sdAvailable = false;
bool lastSDState = false;

unsigned long lastCheck = 0;
unsigned long lastWrite = 0;

const unsigned long checkInterval = 1000;  // cek SD tiap 1 detik
const unsigned long writeInterval = 1000;  // tulis data tiap 2 detik

// Parameter kurva untuk CO2
const float m_CO2 = -1.33;
const float b_CO2 = 2.6;
const float coefficientA = 20.44083;
const float coefficientB = -0.656363;

// Fungsi untuk menghitung resistansi sensor
float getSensorResistance(int sensorValue) {
    float v = (sensorValue / 1023.0) * Vcc;
    return ((Vcc - v) / v) * RL;
}

// Fungsi untuk menghitung PPM CO2 dari rasio R_s/R_o
float calculateCO2PPM(float ratio) {
    return pow(10, (m_CO2 * log10(ratio) + b_CO2));
}

#define NUM_READINGS_CO2 20
int readings_CO2[3][NUM_READINGS_CO2];
int readIndex_CO2[3] = {0, 0, 0};
int total_CO2[3] = {0, 0, 0};
float average_CO2[3] = {0, 0, 0};
float lastFiltered_CO2[3] = {0, 0, 0};
const float alpha = 0.1;
const float noiseThreshold = 1.0;

#define NUM_READINGS_CO 10
int readings_CO[3][NUM_READINGS_CO];
int readIndex_CO[3] = {0, 0, 0};
float total_CO[3] = {0, 0, 0};
float average_CO[3] = {0, 0, 0};

// Tambahan untuk stabilisasi PM2.5
const int FILTER_SIZE = 15;
float filterBuffers[3][FILTER_SIZE];
float smoothedPM25[3] = {0, 0, 0};  // Untuk menyimpan nilai rata-rata

const float ALPHA_PM25 = 1.0;  // Faktor smoothing untuk EMA

// Fungsi sorting untuk median filter
void sortBuffer(float* buffer, int size) {
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (buffer[j] > buffer[j + 1]) {
                float temp = buffer[j];
                buffer[j] = buffer[j + 1];
                buffer[j + 1] = temp;
            }
        }
    }
}

// Fungsi median filter
float applyMedianFilter(float* buffer, float newValue) {
    // Geser nilai lama ke belakang
    for (int i = FILTER_SIZE - 1; i > 0; i--) {
        buffer[i] = buffer[i - 1];
    }
    buffer[0] = newValue;

    // Salin data untuk disortir
    float sortedBuffer[FILTER_SIZE];
    for (int i = 0; i < FILTER_SIZE; i++) {
        sortedBuffer[i] = buffer[i];
    }

    // Urutkan data
    sortBuffer(sortedBuffer, FILTER_SIZE);

    // Ambil nilai median
    return sortedBuffer[FILTER_SIZE / 2];
}

// Fungsi untuk mendeteksi keberadaan SD card
void checkSDCardStatus(unsigned long currentTime) {
  if (currentTime - lastCheck >= checkInterval) {
    lastCheck = currentTime;

    sdAvailable = SD.begin(chipSelect);

    if (sdAvailable != lastSDState) {
      lastSDState = sdAvailable;

      if (sdAvailable) {
        Serial.println("SD card tersedia.");
      } else {
        Serial.println("SD card tidak terdeteksi.");
      }
    }

    // Atur LED status
    digitalWrite(LED_HIJAU, sdAvailable ? HIGH : LOW);
    digitalWrite(LED_MERAH, sdAvailable ? LOW : HIGH);
  }
}

void writeToSDCard(float co[3], float co2[3], float pm25[3]) {
  if (!sdAvailable) return;

  digitalWrite(LED_KUNING, HIGH); // Indikator menulis

  DateTime now = rtc.now();  // Ambil waktu dari RTC

  // Format nama file: "Mei-2025.csv"
  String filename = String(namaBulan[now.month()]) + "-" + String(now.year()) + ".csv";
  filename.replace(" ", ""); // Hilangkan spasi jika ada
  filename = filename + '\0';

  char fileChar[20];
  filename.toCharArray(fileChar, sizeof(fileChar));

  bool fileExists = SD.exists(fileChar);  // Cek apakah file sudah ada
  dataFile = SD.open(fileChar, FILE_WRITE);

  if (dataFile) {
    // Jika file baru, tulis header
    if (!fileExists) {
      dataFile.println("Tanggal;Waktu;MQ-7 A;MQ-7 B;MQ-7 C;MQ-135 A;MQ-135 B;MQ-135 C;Satuan;GP2Y1010AU0F-A;GP2Y1010AU0F-B;GP2Y1010AU0F-C;Satuan;Nilai");
    }

    // Format tanggal dan waktu
    char tanggal[11]; // HH/BB/TTTT
    sprintf(tanggal, "%02d/%02d/%04d", now.day(), now.month(), now.year());

    char waktu[9]; // JJ:MM:DD
    sprintf(waktu, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

    // Tulis semua data sensor
    // CO dan CO2
    dataFile.print(tanggal); 
    dataFile.print(';');
    dataFile.print(waktu);   
    dataFile.print(';');
    dataFile.print(co[0], 1); 
    dataFile.print(';');
    dataFile.print(co[1], 1); 
    dataFile.print(';');
    dataFile.print(co[2], 1); 
    dataFile.print(';');
    dataFile.print(co2[0], 1); 
    dataFile.print(';');
    dataFile.print(co2[1], 1); 
    dataFile.print(';');
    dataFile.print(co2[2], 1);
    dataFile.print(';');
    dataFile.print(" ppm");
    dataFile.print(';');
    // PM2.5
    dataFile.print(pm25[0], 1);
    dataFile.print(';'); 
    dataFile.print(pm25[1], 1);
    dataFile.print(';'); 
    dataFile.print(pm25[2], 1);
    dataFile.print(';'); 
    dataFile.print(" ug/m3");
    dataFile.print(';');
    // Status Relay
    if (hasWrittenOffStatus == false) {
      dataFile.println("Aktif");
    } else if (hasWrittenOffStatus == true){
      dataFile.println("Nonaktif");
      hasWrittenOffStatus = false;
    }


    dataFile.close();
    Serial.print("Data ditulis ke: ");
    Serial.println(fileChar);
  } else {
    Serial.println("Gagal membuka file CSV.");
  }

  digitalWrite(LED_KUNING, LOW);
}

void setup() {
  Serial.begin(9600);
  if (!rtc.begin()) {
    Serial.println("RTC tidak terdeteksi!");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC kehilangan daya. Waktu perlu diatur ulang.");
  }
  for (int i = 0; i < 3; i++) {
      pinMode(ledPowers[i], OUTPUT);
  }

  // [TAMBAHAN] Setup untuk relay dan flame sensor
  pinMode(flamesensor, INPUT);
  pinMode(relayA, OUTPUT);
  pinMode(LED_HIJAU, OUTPUT);
  pinMode(LED_MERAH, OUTPUT);
  pinMode(LED_KUNING, OUTPUT);

  pinMode(chipSelect, OUTPUT); // sangat penting untuk Mega
  digitalWrite(chipSelect, HIGH); // default tidak aktifkan CS
  digitalWrite(relayA, HIGH); // Pastikan relay awalnya mati

  /* // Kalibrasi R_o untuk setiap sensor
  for (int i = 0; i < 3; i++) {
      long sumRs = 0;
      int numSamples = 1000;
      for (int j = 0; j < numSamples; j++) {
          int sensorValue = analogRead(analogPins_CO2[i]);
          sumRs += getSensorResistance(sensorValue);
          delay(50);
      }
      Ro_CO2[i] = sumRs / numSamples;
  }*//*
  // Kalibrasi R_o untuk setiap sensor
  for (int i = 0; i < 3; i++) {
      Ro_CO2[i] = 0;
  }
    
  for (int sample = 0; sample < SAMPLES; sample++) {
      for (int i = 0; i < 3; i++) {
          int sensorValue = analogRead(analogPins_CO2[i]);
          Ro_CO2[i] += getSensorResistance(sensorValue) / SAMPLES;
      }
        
      // Hitung waktu yang tersisa
      int remainingTime = CALIBRATION_TIME - (sample * SAMPLE_INTERVAL);
      int minutes = remainingTime / 60;
      int seconds = remainingTime % 60;
       
      // Animasi progress bar di OLED
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.print("Warm Up...");
      display.setCursor(0, 10);
      display.print("Time Left: ");
      display.print(minutes);
      display.print("m ");
      display.print(seconds);
      display.print("s");
        
      int progress = (sample * 100) / SAMPLES;
      display.drawRect(10, 40, 100, 10, SSD1306_WHITE);
      display.fillRect(10, 40, progress, 10, SSD1306_WHITE);
      display.display();
        
      delay(SAMPLE_INTERVAL * 1000);
  }*/
    
 
  
}

void loop() {
  unsigned long currentMillis = millis();
  float filteredDensities[3];
  checkSDCardStatus(currentMillis);
  
  for (int i = 0; i < 3; i++) {
      //CO2
      int sensorValue_CO2 = analogRead(analogPins_CO2[i]);
      Rs_CO2[i] = getSensorResistance(sensorValue_CO2);
      ratio_CO2[i] = Rs_CO2[i] / Ro_CO2[i];
      float ppmCO2 = calculateCO2PPM(ratio_CO2[i]);

      total_CO2[i] = total_CO2[i] - readings_CO2[i][readIndex_CO2[i]];
      readings_CO2[i][readIndex_CO2[i]] = ppmCO2;
      total_CO2[i] = total_CO2[i] + readings_CO2[i][readIndex_CO2[i]];
      readIndex_CO2[i] = (readIndex_CO2[i] + 1) % NUM_READINGS_CO2;
      float movingAvgCO2 = total_CO2[i] / NUM_READINGS_CO2;

      float smoothedCO2 = (alpha * movingAvgCO2) + ((1 - alpha) * lastFiltered_CO2[i]);

      if (abs(smoothedCO2 - lastFiltered_CO2[i]) > noiseThreshold) {
          lastFiltered_CO2[i] = smoothedCO2;
      }

      average_CO2[i] = lastFiltered_CO2[i] + 65;

      
      //CO
      int sensorValue_CO = analogRead(analogPins_CO[i]);
      Vout_CO[i] = (sensorValue_CO / 1023.0) * Vcc;
      ratio_CO[i] = (Vcc - Vout_CO[i]) / Vout_CO[i];
      ppmCO[i] = coefficientA * pow(ratio_CO[i], coefficientB);
      
      total_CO[i] = total_CO[i] - readings_CO[i][readIndex_CO[i]];
      readings_CO[i][readIndex_CO[i]] = ppmCO[i];
      total_CO[i] = total_CO[i] + readings_CO[i][readIndex_CO[i]];
      readIndex_CO[i] = (readIndex_CO[i] + 1) % NUM_READINGS_CO;
      average_CO[i] = total_CO[i] / NUM_READINGS_CO;
      
      //PM2.5
      digitalWrite(ledPowers[i], LOW);
        delayMicroseconds(samplingTime);

        long totalReading = 0;
        const int NUM_SAMPLES = 50;
        
        for (int j = 0; j < NUM_SAMPLES; j++) {
            totalReading += analogRead(measurePins[i]);
            delayMicroseconds(10);
        }

        float voMeasured = (float)totalReading / NUM_SAMPLES;
        delayMicroseconds(deltaTime);
        digitalWrite(ledPowers[i], HIGH);
        delayMicroseconds(sleepTime);

        float calcVoltage = voMeasured * (5.0 / 1024.0);
        float dustDensity = 170 * calcVoltage - 0.1;

        // Terapkan median filter
        float medianFilteredPM25 = applyMedianFilter(filterBuffers[i], dustDensity);

        // Terapkan Exponential Moving Average (EMA)
        smoothedPM25[i] = (ALPHA_PM25 * medianFilteredPM25) + ((1 - ALPHA_PM25) * smoothedPM25[i]);

        filteredDensities[i] = smoothedPM25[i];

      if (i == 0) { 
        filteredDensities[i] -= 610;//134;//44;
        if (filteredDensities[i] < 0) filteredDensities[i] = 0;
      }
      if (i == 1) { 
        filteredDensities[i] -= 55;//53.4;
        if (filteredDensities[i] < 0) filteredDensities[i] = 0;
      }
      if (i == 2) { 
        filteredDensities[i] -= 0;//130;//51;
        if (filteredDensities[i] < 0) filteredDensities[i] = 0;
      }
      if (i == 0) { 
        average_CO[i] -= 4;
        if (average_CO[i] < 0) average_CO[i] = 0;
      }
      if (i == 1) { 
        average_CO[i] -= 4;
        if (average_CO[i] < 0) average_CO[i] = 0;
      }
      if (i == 2) { 
        average_CO[i] -= 0;
        if (average_CO[i] < 0) average_CO[i] = 0;
      }
  }

 
    
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.println("FILTER 1");
    Serial.print("CO    :"); Serial.println(average_CO[0],0);
    Serial.print("CO2   :"); Serial.println(average_CO2[0],0); 
    Serial.print("PM2.5 :"); Serial.println(filteredDensities[0],0);
    Serial.println("");
    
    int flamevalue = digitalRead(flamesensor);
    Serial.println("FILTER 2");
    Serial.print("CO    :"); Serial.println(average_CO[1],0);
    Serial.print("CO2   :"); Serial.println(average_CO2[1],0); 
    Serial.print("PM2.5 :"); Serial.println(filteredDensities[1],0);
    Serial.println(flamevalue);

    Serial.println("FILTER 3");
    Serial.print("CO    :"); Serial.println(average_CO[2],0);
    Serial.print("CO2   :"); Serial.println(average_CO2[2],0); 
    Serial.print("PM2.5 :"); Serial.println(filteredDensities[2],0);
    Serial.println("======================================================");
  }


  int flamevalue = digitalRead(flamesensor);
  if (flamevalue == 0){
    digitalWrite(relayA, LOW);
    flamestate = false; 
  } else if(average_CO2[0] >= 700){
    digitalWrite(relayA, LOW);
  } else if (!flamestate && average_CO2[0] >= 500) {
    digitalWrite(relayA, LOW); // Relay AKTIF (active low)
  } else if (average_CO2[0] <= 499) {
    digitalWrite(relayA, HIGH); // Relay MATI
    flamestate = true;
  }

  // Ambil status relay saat ini
  bool currentRelayState = digitalRead(relayA);

  // Logika penulisan status ke SD card:
  if (currentRelayState == LOW) { 
    // Case 1: Relay AKTIF -> Tulis "Aktif" setiap 1 detik
    if (currentMillis - lastWrite >= writeInterval) {
      writeToSDCard(average_CO, average_CO2, filteredDensities);
      lastWrite = currentMillis;
      hasWrittenOffStatus = false; // Reset flag untuk status mati
    }
  } 
  else { 
    // Case 2: Relay MATI -> Tulis "Mati" hanya 1 kali
    if (!hasWrittenOffStatus) {
      writeToSDCard(average_CO, average_CO2, filteredDensities);
      hasWrittenOffStatus = true; // Set flag agar tidak menulis lagi
      lastWrite = currentMillis; // Reset timer
    }
  }

  lastRelayState = currentRelayState; // Update status relay
}
