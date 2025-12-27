#include <SD.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include "DHT.h"

// Known resistor value in voltage divider circuit
const float R1 = 10000;
// Variables for temperature calculation
float logR2, R2, T, Tc;
// Output Voltage from the thermistor voltage divider
int Vo;
// Steinhart-Hart coefficients for thermistor
const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
// Temperature control
const int relayDeactivationTemp = 25;
const int relayActivationTemp = 5;


// DHT22 Humidity Sensor constants 
#define DHTPIN 9
#define DHTTYPE DHT22
DHT myDHT(DHTPIN, DHTTYPE);

// SD card constant
const int SDchipSelect = 8;
// Analog pins
const int ExteriorTempPin = 1;
const int InteriorTempPin = 0;
const int PressurePin = 2;
// Digital Pins
const int CameraPin = 5;
const int CutdownPin = 2;
const int HeatRelayPin = 8;
const int BuzzerPin = 6;

// Pressure sensor constants
const float maxPressure = 150.0;  // Maximum measurable pressure for sensor
const float sensorVoltageRange = 5.0;  // Sensor voltage range
const float sensorMaxValue = 1023.0;  // AnalogRead max value for 10-bit ADC
const int CameraActivationPressure = 60;  // Pressure value where Camera will start to take photos
const int CutdownActivationPressure = -30; // Sends voltage singnal to Cutdown and drops it
const int BuzzerActivationTime = 10800;
const int CameraActivationTime = 300; 
// Sets SCL to the TX(Transmits Data) and sets SDA to the RX(Receive and read data)
SoftwareSerial s_serial(SCL, SDA);   
#define sensor s_serial

// Command array to request data from the CO₂ sensor
const unsigned char cmd_get_sensor[] = { 0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };

// Results variables
float pressure;
int time = 0;
float TempInterior;
float TempExterior;
// float TempHumiditySensor;
float humidity;     
float TempBlack;
float TempWhite; 

// Booleans to check activation status
bool HeatPadStatus = false;
bool CameraStatus = false;
bool CutdownStatus = false;
// bool BuzzerStatus = false;

// File to store data
File dataFile;

void setup() {

  sensor.begin(9600);    // Initialize CO₂ sensor at 9600 baud rate
  Serial.begin(9600);

  // Initialize DHT22 Humidity sensor
  myDHT.begin();    

  // Sets Camera, Relay and Cutdown, Buzzer Pins as outputs
  pinMode(HeatRelayPin, OUTPUT);
  pinMode(CameraPin, OUTPUT);
  pinMode(CutdownPin, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);

  // Turn off heat pad
  digitalWrite(HeatRelayPin, LOW);

   // Turn off camera
  digitalWrite(CameraPin, LOW);

  // Leave Cutdown attachted 
  digitalWrite(CutdownPin, LOW);


  // Initialize SD card
  if (!SD.begin(SDchipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");

  // Opens and clears file
  dataFile = SD.open("templog.txt", FILE_WRITE | O_TRUNC);
  // Initialize data log file
  if (dataFile) {
    dataFile.println("Time(s)  Pressure(kPa)  Temp Interior(C)  Temp Exterior(C)   TempBlack(°C)        TempWhite(°C)       Humidity(%)    HeatPad   Camera     Cutdown");
    dataFile.close();
  }
  delay(5000);
}

void loop() {
  time = time + 5;

  TempInterior = 0.713 * calcTemp(InteriorTempPin) - 6.95;
  TempExterior = 0.713 * calcTemp(ExteriorTempPin) - 6.95;

  TempBlack = calcTemp(4);
  TempWhite = calcTemp(3);

  humidity = getHumidity();

  checkTemp(TempInterior);
  pressure = calcPressure();

  // turn on buzzer after three hours
  if (time > BuzzerActivationTime){
    digitalWrite(BuzzerPin, HIGH);
    // BuzzerStatus = true;
  }

  // Turns camera on or off depending on pressure
  controlCameraByPressure(pressure, time);
  // Triggers Cutdown if it reaches certain pressure
  controlCutdown(pressure, time);
  
  // Display readings on Serial Monitor
  displayReadings();

  // Log data to SD card
  logDataToSD();

  delay(5000);
}

// Calculates the pressure from input voltage
float calcPressure(){
  int sensorValue = analogRead(PressurePin);   // Read the analog value from pressure sensor
  float pressureVoltage = (sensorValue / sensorMaxValue) * sensorVoltageRange;  // Convert to voltage
  float pressure = (pressureVoltage / sensorVoltageRange) * maxPressure;  // Convert to pressure
  pressure = 2.68 * pressure - 95.4;
  return pressure;
}

// Calculates temperature in Celcius
float calcTemp(int pin){
  Vo = analogRead(pin);
  // Calculate the resistance of the thermistor using the voltage divider formula
  R2 = R1 * (1023.0 / Vo - 1.0);
  // Calculates resistance of thermister
  logR2 = log(R2);
  // Apply the Steinhart-Hart equation to convert resistance to temperature (in Kelvin)
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  return T;
}

float getHumidity(){
  float hum = myDHT.readHumidity();

  if (isnan(hum)){
    Serial.println("Failed to Read Data on DHT22");
    return -1000;
  }
  return hum;
}

// Turns Heat Relay on our off depending on temperature
void checkTemp(float T){
  // Turns Heat Relay off]
  if (T > relayDeactivationTemp){
    Serial.println("Heat Relay:  Off");
    digitalWrite(HeatRelayPin, LOW);
    HeatPadStatus = false;
  }
  // Turns Heat Relay on 
  else if(T < relayActivationTemp && T != -201.71){
    Serial.println("Heat Relay:  ON");
    digitalWrite(HeatRelayPin, HIGH);
    HeatPadStatus = true;
  }
}


// Controls the camera based on the pressure threshold
void controlCameraByPressure(float pressure, int time){

  if ((pressure < CameraActivationPressure && pressure != -95.4 && (time % 60 == 0)) || (time % CameraActivationTime == 0)){
    digitalWrite(CameraPin, HIGH);
    CameraStatus = true;
  }
  else{
    digitalWrite(CameraPin, LOW);
    CameraStatus = false;
  }
}

// Controls the Cutdown mechanism based on the pressure threshold
void controlCutdown(float pressure, int time){
  if ((pressure < CutdownActivationPressure && pressure != -95.4 && time > 7200) || time > 10800) {
    digitalWrite(CutdownPin, HIGH);
    CutdownStatus = true;
  }
  else{
    digitalWrite(CutdownPin, LOW);
  }
}

// Prins the Readings from all sensors into the SerialMonitor
void displayReadings() {
  // Print the header
  Serial.println("Time (s)\tPressure (kPa)\tTemp Interior (C)\tTemp Exterior (C)\tTemp Black (C)\tTempWhite (°C)\tHumidity (%)");

  // Print values corresponding to headers
  Serial.print(time);
  Serial.print("\t\t");
  Serial.print(pressure);
  Serial.print("\t\t\t");
  Serial.print(TempInterior);
  Serial.print("\t\t\t");
  Serial.print(TempExterior);
  Serial.print("\t\t\t");
  Serial.print(TempBlack);
  Serial.print("\t\t\t");
  Serial.print(TempWhite);  
  Serial.print("\t\t\t");
  Serial.println(humidity);
}

void logDataToSD() {
  // Open the file for writing
  dataFile = SD.open("templog.txt", FILE_WRITE);

  if (dataFile) {
    // Write the data values with consistent spacing
    dataFile.print(time); dataFile.print("         "); 
    dataFile.print(pressure); dataFile.print("             ");
    dataFile.print(TempInterior); dataFile.print("             ");
    dataFile.print(TempExterior); dataFile.print("           ");
    dataFile.print(TempBlack); dataFile.print("               ");
    dataFile.print(TempWhite); dataFile.print("                ");
    dataFile.print(humidity); dataFile.print("         ");
    dataFile.print(HeatPadStatus ? "ON " : "OFF"); dataFile.print("       ");
    dataFile.print(CameraStatus ? "ON " : "OFF"); dataFile.print("       ");
    dataFile.println(CutdownStatus ? "Yes " : "No"); 
    // dataFile.println(BuzzerStatus ? "ON" : "OFF");

    dataFile.close();
    Serial.println("Data written to SD card.");
  } 
  else {
    // If the file didn't open, print an error
    Serial.println("Error opening templog.txt");
  }
}
