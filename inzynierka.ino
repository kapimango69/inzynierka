#include "BluetoothSerial.h"
#include "esp_timer.h"
#include "SPI.h"
#include "FS.h"
#include "SD.h"
#include <Wire.h>
#include<numeric>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_SH110X.h>


#define BLUETOOTH_RED 16
#define BLUETOOTH_BLUE 17
#define i2c_address_oled 0x3c
#define SCREEN_WIDTH 128 // w px
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // definicja wyswietlacza

class SensorsData {
private:
  int time;
  int x;
  int y;
  int z;
  int bpm;
  int spo2;
  int breath;
  char buffer[200];
public:
  SensorsData() {
    this->time = 0;
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->bpm = 0;
    this->spo2 = 0;
    this->breath = 0;
    buffer_update();
  }
  int *get_x_pointer() {
    return &x;
  }
  int *get_y_pointer() {
    return &y;
  }
  int *get_z_pointer() {
    return &z;
  }
  int *get_bpm_pointer() {
    return &bpm;
  }
  int *get_spo2_pointer() {
    return &spo2;
  }
  int *get_breath_pointer() {
    return &breath;
  }
  void time_update(int seconds) {
    this->time += seconds;
  }
  void reset_time() {
    this->time = 0;
  }
  void buffer_update() {
    int size = sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d\n", time, x, y, z, bpm, spo2, breath) + 2;
    sprintf(buffer, "%d%d,%d,%d,%d,%d,%d,%d\n", size, time, x, y, z, bpm, spo2, breath);
  }
  char *getBuffer() {
    return buffer;
  }
};

class ADXL345 {
private:
  int ADXL345_POWER_CTL = 0x2D;
  int ADXL345_ADDRESS = 0x53;
  int ADXL345_DATA_FORMAT = 0x31;
  int ADXL345_ACT_INACT_CTL = 0x27;
  int MISO = 12;
  int MOSI = 13;
  int SCK = 14;
  int CS = 15;
  SPIClass hspi;
  byte _buff[6];
public:
  ADXL345()
    : hspi(HSPI) {}
  void init() {
    hspi.begin(SCK, MISO, MOSI, CS);
    hspi.setDataMode(SPI_MODE3);
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);
    powerOn();
    delay(500);
    setRangeSetting(16);
    delay(500);
    setRegisterBit(ADXL345_DATA_FORMAT, 6, 0);
    delay(500);
  }
  void readAcc(int *x, int *y, int *z) {
    readFrom(0x32, 6, _buff);
    *x = (int16_t)((((int)_buff[1]) << 8) | _buff[0]);
    *y = (int16_t)((((int)_buff[3]) << 8) | _buff[2]);
    *z = (int16_t)((((int)_buff[5]) << 8) | _buff[4]);
  }
private:
  void readFrom(byte _reg_adress, int num, byte _buff[]) {
    char _address = 0x80 | _reg_adress;
    if (num > 1) {
      _address = _address | 0x40;
    }
    digitalWrite(CS, LOW);
    hspi.transfer(_address);
    for (int i = 0; i < num; i++) {
      _buff[i] = hspi.transfer(0x00);
    }
    digitalWrite(CS, HIGH);
  }
  void powerOn() {
    writeTo(ADXL345_POWER_CTL, 0);
    writeTo(ADXL345_POWER_CTL, 16);
    writeTo(ADXL345_POWER_CTL, 8);
  }
  void writeTo(byte _reg_address, byte _val) {
    digitalWrite(CS, LOW);
    hspi.transfer(_reg_address);
    hspi.transfer(_val);
    digitalWrite(CS, HIGH);
  }
  void setRangeSetting(int val) {
    byte _s;
    byte _b;
    switch (val) {
      case 2:
        _s = B00000000;
        break;
      case 4:
        _s = B00000001;
        break;
      case 8:
        _s = B00000010;
        break;
      case 16:
        _s = B00000011;
        break;
      default:
        _s = B00000000;
        break;
    }
    readFrom(ADXL345_DATA_FORMAT, 1, &_b);
    _s |= (_b & B11101100);
    writeTo(ADXL345_DATA_FORMAT, _s);
  }
  void setRegisterBit(byte regAdress, int bitPos, bool state) {
    byte _b;
    readFrom(regAdress, 1, &_b);
    if (state) {
      _b |= (1 << bitPos);
    } else {
      _b &= ~(1 << bitPos);
    }
    writeTo(regAdress, _b);
  }
};

class MAX30102 {

private:
  MAX30105 particleSensor;

//------------------------Define I2C pins---------------------------------
#define I2C_SDA 21
#define I2C_SCL 22

  //------------------------Variables to calculate SpO2---------------------
  double avered = 0;
  double aveir = 0;
  double sumirrms = 0;
  double sumredrms = 0;
  int i_val = 0;
  int Num = 100;  //calculate SpO2 by this sampling interval

  int oxygen;
  double ESpO2 = 95.0;  //initial value of estimated SpO2
  double FSpO2 = 0.7;   //filter factor for estimated SpO2
  double frate = 0.95;  //low pass filter for IR/red LED value to eliminate AC component

#define TIMETOBOOT 3000  // wait for this time(msec) to output SpO2
#define FINGER_ON 7000   // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 0.0
#define MINIMUM_BPM 0
#define USEFIFO
#define RATE_SIZE 15  //Increase this for more averaging. 4 is good.

  //------------------------Variables to calculate HR---------------------
  byte rates[RATE_SIZE];  //Array of heart rates
  byte rateSpot = 0;
  long lastBeat = 0;  //Time at which the last beat occurred
  float beatsPerMinute;
  int beatAvg;
  long irValue;

public:
  MAX30102()
    : particleSensor() {}

  void init() {
    //Setup I2C communication
    Wire.begin(I2C_SDA, I2C_SCL);

    //Begin MAX30102 sensor
    particleSensor.begin();

    //-----------------Setup MAX30102 for best results--------------------------------
    byte ledBrightness = 20;  //Options: 0=Off to 255=50mA
    byte sampleAverage = 8;   //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2;         //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    int sampleRate = 400;     //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411;     //Options: 69, 118, 215, 411
    int adcRange = 4096;      //Options: 2048, 4096, 8192, 16384

    //particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
    particleSensor.setup();
    particleSensor.enableDIETEMPRDY();
  }

  void calc_HR() {
    //------------------------Calculate HR----------------------------
    irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true) {
      //Sensed a beat
      long delta = millis() - lastBeat;
      lastBeat = millis();

      //Calculate BPM
      beatsPerMinute = 60.0 / (delta / 1000.0);

      //Moving average
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;  //Store this reading in the array
        rateSpot %= RATE_SIZE;                     //Wrap variable

        //Take average of readings
        beatAvg = 0;
        // for (byte x = 0; x < RATE_SIZE; x++)
        //   beatAvg += rates[x];
        beatAvg = std::accumulate(rates,rates+RATE_SIZE,beatAvg);
        beatAvg /= RATE_SIZE;
      }
    }
  }

  void calc_SpO2() {
    //------------------------Calculate SpO2----------------------------
    uint32_t ir, red, green;
    double fred, fir;
    double SpO2 = 0;  //raw SpO2 before low pass filtered

    particleSensor.check();  //Check the sensor, read up to 3 samples

    while (particleSensor.available()) {
      // Get data
      red = particleSensor.getFIFORed();
      ir = particleSensor.getFIFOIR();
      //
      i_val++;
      fred = (double)red;
      fir = (double)ir;
      avered = avered * frate + (double)red * (1.0 - frate);  //average red level by low pass filter
      aveir = aveir * frate + (double)ir * (1.0 - frate);     //average IR level by low pass filter
      sumredrms += (fred - avered) * (fred - avered);         //square sum of alternate component of red level
      sumirrms += (fir - aveir) * (fir - aveir);              //square sum of alternate component of IR level

      if (millis() > TIMETOBOOT) {
        // SpO2 upper & lower limit
        if (ESpO2 <= -1) {
          ESpO2 = 0;
        }
        if (ESpO2 > 100) {
          ESpO2 = 100;
        }
        oxygen = ESpO2;
      }
      // Calculate SpO2 from equation
      if ((i_val % Num) == 0) {
        double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);  //SpO2 Ratio
        SpO2 = -23.3 * (R - 0.4) + 100;                                    //SpO2 from Sample Calibration Curve
        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;                      //low pass filter

        // Reset sum values
        sumredrms = 0.0;
        sumirrms = 0.0;
        i_val = 0;
        break;
      }
      particleSensor.nextSample();  //Move to next sample
    }
  }

  void readVal(int *bpm, int *spo2) {
    //------------------------Show values----------------------------
    if (irValue > FINGER_ON)  //If a finger is detected
    {
      *bpm = beatAvg;
      *spo2 = oxygen;
    } else {
      beatAvg = MINIMUM_BPM;
      ESpO2 = MINIMUM_SPO2;  //indicator for finger detached
      *bpm = beatAvg;
      *spo2 = 0;
      Serial.println("Please place your finger");
    }
  }
};

class BreathSensor {

private:
// ---------------------Defines and variables------------------------
#define ADCpin_Sens 39         // ADC pin number
#define MINIMUM_RESPIRATORY 0  // Reset respiratory rate value after crash
#define RATE_SIZE2 8           //Variable for moving average - (Increase this for more averaging. 4 is good.)

  const int Min_SensVal = 700;   //Minimal value of respiratory samples
  const int Max_SensVal = 3000;  //Maximum value of respiratory samples

  const double frate = 0.95;       //Weight of low pass filter
  const int sampling_data3 = 100;  //Change sampling data in milliseconds

  int16_t AC_Max = 2;   //Initial value of local max
  int16_t AC_Min = -2;  //Initial value of local min

  //-----------Variables for writing respiratory data samples-----------
  int sensVal = 0;
  bool isSensTrue = false;
  unsigned long previousMillis3 = 0;

  //-------------Variables for checkForBreath() function----------------
  double avesens = 0;
  double avesens_prev;
  int32_t avg_reg = 0;
  int16_t testEstimator;
  int16_t Signal_min = 0;
  int16_t Signal_max = 0;
  int16_t posEdge = 0;
  int16_t negEdge = 0;

  // --------------Variables for moving average calculation-------------
  byte rates2[RATE_SIZE2];  //Array of breath rates2
  byte rates2pot = 0;
  long lastBreath = 0;  //Time at which the last beat occurred
  float breathsPerMinute;
  int breathAvg;

public:

  int16_t averageEstimator(int32_t *p, uint16_t x) {
    *p += ((((long)x << 15) - *p) >> 4);
    return (*p >> 15);
  }


  bool checkForBreath(int32_t sample) {
    bool breathDetected = false;

    testEstimator = averageEstimator(&avg_reg, sensVal);  //DC Estimator of respiratory data sample
    double minus_Est = sensVal - testEstimator;           //AC value of respiratory data sample

    avesens = avesens * frate + minus_Est * (1.0 - frate);  //Low pass filter of AC value

    //  Detect positive zero crossing (rising edge)
    if ((avesens_prev < 0) & (avesens >= 0)) {
      AC_Max = Signal_max;  //Adjust our AC max and min
      AC_Min = Signal_min;

      posEdge = 1;
      negEdge = 0;
      Signal_max = 0;

      if ((AC_Max - AC_Min) >= 1 & (AC_Max - AC_Min) < 100)  //Adjust values to work with sensor
      {
        breathDetected = true;
        Serial.println("Breath detected");
      }
    }

    //  Detect negative zero crossing (falling edge)
    if ((avesens_prev > 0) & (avesens <= 0)) {
      posEdge = 0;
      negEdge = 1;
      Signal_min = 0;
    }

    //  Find Maximum value in positive cycle
    if (posEdge & (avesens > avesens_prev)) {
      Signal_max = avesens;
    }

    //  Find Minimum value in negative cycle
    if (negEdge & (avesens < avesens_prev)) {
      Signal_min = avesens;
    }

    return (breathDetected);
  }


  void calc_RespRate() {
    if ((millis() - previousMillis3) > sampling_data3) {
      avesens_prev = avesens;

      sensVal = analogRead(ADCpin_Sens);

      //Check if value of respiratory data sample is real
      if (sensVal > Max_SensVal || sensVal < Min_SensVal) {
        isSensTrue = false;
      } else {
        isSensTrue = true;
      }

      if (checkForBreath(sensVal) == true) {
        //Sensed a breath
        long delta = millis() - lastBreath;
        lastBreath = millis();

        //Calculate Respiratory rate
        breathsPerMinute = 60.0 / (delta / 1000.0);

        //Moving average
        if (breathsPerMinute < 60 && breathsPerMinute > 0) {
          rates2[rates2pot++] = (byte)breathsPerMinute;  //Store this reading in the array
          rates2pot %= RATE_SIZE2;                       //Wrap variable

          //Take average of readings
          breathAvg = 0;
          // for (byte x = 0; x < RATE_SIZE2; x++)
          //   breathAvg += rates2[x];
          breathAvg = std::accumulate(rates2,rates2+RATE_SIZE2,breathAvg);
          breathAvg /= RATE_SIZE2;
        }
      }
      previousMillis3 = millis();
    }
  }


  void readVal(int *breath) 
  {
    if (isSensTrue == true){
      *breath = breathAvg;
    }
    else{
      breathAvg = MINIMUM_RESPIRATORY;
      *breath = breathAvg;
      Serial.println("Breath sensor disconnected");
    }
  }

};


BluetoothSerial SerialBT;
SensorsData sensorsData = SensorsData();
ADXL345 adxl345 = ADXL345();
MAX30102 max30102 = MAX30102();
BreathSensor breathsensor = BreathSensor();

int card_seconds = 1;
int card_time = 1000000 * card_seconds;

unsigned long previousMillis = 0;
const long sendingDataInterval =  card_time / 1000;

unsigned long bufferPreviousUpdateTime = 0;
const long bufferUpdateInterval_milis = 1000;

//-----------------Additional variables-----------------
unsigned long previousMillis2 = 0;
int sampling_data = 100;  //change sampling data in milliseconds

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");
  init_display();
  adxl345.init();
  max30102.init();

  pinMode(BLUETOOTH_RED, OUTPUT);
  pinMode(BLUETOOTH_BLUE, OUTPUT);
  digitalWrite(BLUETOOTH_RED, LOW);
  digitalWrite(BLUETOOTH_BLUE, LOW);
}
void cardSD(SensorsData *data) {
  if (!SD.begin()) {
    Serial.println("Card Mount Failed.");
    return;
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached.");
    return;
  }
  appendFile(SD, "/sensors_data.txt", data->getBuffer());
}

void loop() {

  breathsensor.calc_RespRate();
  max30102.calc_HR();
  max30102.calc_SpO2();

  if ((millis() - previousMillis2) > sampling_data) {
    Serial.print(*sensorsData.get_x_pointer());
    Serial.print(", ");
    Serial.print(*sensorsData.get_y_pointer());
    Serial.print(", ");
    Serial.print(*sensorsData.get_z_pointer());
    Serial.print(", ");
    Serial.print(*sensorsData.get_bpm_pointer());
    Serial.print(", ");
    Serial.print(*sensorsData.get_spo2_pointer());
    Serial.print(", ");
    Serial.println(*sensorsData.get_breath_pointer());
    previousMillis2 = millis();
  }

  if (!SerialBT.hasClient()) {
    digitalWrite(BLUETOOTH_RED, LOW);
    digitalWrite(BLUETOOTH_BLUE, LOW);
  }
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil(';');
    Serial.print("Recieved command: ");
    Serial.println(command);
    if (command == "1") {
      digitalWrite(BLUETOOTH_RED, HIGH);
    }
    if (command == "0") {
      digitalWrite(BLUETOOTH_RED, LOW);
    }
    if (command == "Connect") {
      digitalWrite(BLUETOOTH_BLUE, HIGH);
    }
    if (command == "Disconnect") {
      digitalWrite(BLUETOOTH_BLUE, LOW);
    }
  }
  unsigned long currentMillis = millis();
  if (currentMillis - bufferPreviousUpdateTime >= bufferUpdateInterval_milis) {
    bufferPreviousUpdateTime = currentMillis;
    sensorsData.time_update(card_seconds);
    adxl345.readAcc(
      sensorsData.get_x_pointer(),
      sensorsData.get_y_pointer(),
      sensorsData.get_z_pointer());
    max30102.readVal(
      sensorsData.get_bpm_pointer(),
      sensorsData.get_spo2_pointer());
    breathsensor.readVal(
      sensorsData.get_breath_pointer()
    );
    sensorsData.buffer_update();
    update_oled(sensorsData.get_bpm_pointer(), sensorsData.get_spo2_pointer(),sensorsData.get_breath_pointer(),SerialBT.hasClient());
  }

  currentMillis = millis();
  if (currentMillis - previousMillis >= sendingDataInterval) {
    previousMillis = currentMillis;
    cardSD(&sensorsData);
    SerialBT.println(sensorsData.getBuffer());
  }
}


void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending.");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended.");
  } else {
    Serial.println("Append failed.");
  }
  file.close();
}

  void update_oled(int *bpm, int *spo2,int *breath,bool bt)
{
  display.clearDisplay(); // clear bufora
  display.setTextSize(1);  // ustawienie rozmiaru tekstu
  display.setTextColor(SH110X_WHITE);  // wybor koloru (biały/czarny)
  display.setCursor(1,1);  // pozycjonowanie kursora do pisania
  display.println("Dane uzytkownika:");  // wypisywany tekst
  display.setCursor(1,13);  // pozycjonowanie kursora do pisania
  display.print("HR: ");  // druga linijka tekstu
  display.println(String(*bpm));
  display.setCursor(1,25);  // pozycjonowanie kursora do pisania
  display.print("Sat: ");  // druga linijka tekstu
  display.print(String(*spo2));  // druga linijka tekstu
  display.println(" %");  // druga linijka tekstu
  display.setCursor(1,37);  // pozycjonowanie kursora do pisania
  display.print("Bpm: ");  // trzecia linijka tekstu
  display.println(String(*breath));  // trzecia linijka tekstu
  display.setCursor(1,52);  // pozycjonowanie kursora do pisania
  if (bt == true){
    display.println("Bluetooth: o"); 
  }
  else{
    display.println("Bluetooth: x");  // druga linijka tekstu
  }
  display.display();  // wyswietlenie bufora z tekstem
}

void init_display(){
  display.begin(i2c_address_oled,true); // rozpoczecie komunikacji
  display.display(); // wyswietlenie bufora 
  display.clearDisplay(); // clear bufora
  display.setTextSize(1);  // ustawienie rozmiaru tekstu
  display.setTextColor(SH110X_WHITE);  // wybor koloru (biały/czarny)
  display.setCursor(1,1);  // pozycjonowanie kursora do pisania
  display.println("Dane uzytkownika:");  // wypisywany tekst
  display.setCursor(1,13);  // pozycjonowanie kursora do pisania
  display.println("HR: 2137");  // druga linijka tekstu
  display.setCursor(1,25);  // pozycjonowanie kursora do pisania
  display.println("Sat: 100 %");  // druga linijka tekstu
  display.setCursor(1,52);  // pozycjonowanie kursora do pisania
  display.println("Bluetooth: x");  // druga linijka tekstu
  display.display();  // wyswietlenie bufora z tekstem
}