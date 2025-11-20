#include "Arduino.h"
#include "ArduinoJson.h"
#include "Adafruit_INA228.h"

#define USBSWTCH_RELAY 26
#define BCI_OUTPUT_AMOUNT 8 // Amount of BCI outputs to be measured
const int adapterBCI_Aux[] = {34, 36, 38}; // Inputs connected to Adapter BCI's outputs
/*
  adapterBCI_Aux[0] = MOLEX pin 6 = GREY wire
  adapterBCI_Aux[1] = MOLEX pin 7 = YELLOW wire
  adapterBCI_Aux[2] = MOLEX pin 9 = BLUE wire
*/

// currentSensor object based on AdaFruit INA228 library
Adafruit_INA228 currentSensor = Adafruit_INA228();

// Pin arrays of digital and analog pins.
// The pin arrays are ordered so that bridged pins are on the same index
// A0 is bridged with (D)9, A1 with (D)8 etc..
// Size of arrays must be 8
const int analogPins[] =    {A0, A1, A2, A3, A4, A5, A6, A7};
const int digitalPins[] =   {9,  8,  7,  6,  5,  4,  3,  2};



// Ratio of voltage divider, in this case based on 3.84kΩ and 2.16kΩ resistors
const float voltageDividerRatio = 2.741;
// Calibration factor for Arduino ADC
const float calibrationFactor = 1;
// Total factor to multiply with analog readings
const float analogFactor = (5.0 / 1023.0) * voltageDividerRatio * calibrationFactor;
// Timeout in microseconds for pulseIn() for when no pulse is detected
const unsigned long pulseTimeout = 30000; 
// Delay from pulseIn() completion to analogRead() function in microseconds
const unsigned int pulseDelay = 100; 

// Amount of samples taken from each output for averaging
const int signalSampleSize = 2;    

// Current sensor (INA228) configuration; current sensor samples internally
// Time for one current measurement = ConversionTime * AveragingCount
const INA228_ConversionTime INAconversionTime = INA228_TIME_150_us;
const INA228_AveragingCount INAaveragingCount = INA228_COUNT_256;

// Self diagnosis; becomes false in case of malfunction
bool systemOK = true;

void setup() {
  Serial.begin(250000);
  for (int i =0; i < 8; i++){ // Setting pinModes for freq measurements (BCI Outputs)
    pinMode(digitalPins[i], INPUT);
    pinMode(analogPins[i], INPUT);
  }

  for (int i =0; i < 3; i++){ // Setting pinModes for Adapter-BCI Aux inputs
    pinMode(adapterBCI_Aux[i], INPUT);
  }

  pinMode(USBSWTCH_RELAY, OUTPUT);   // Sets the pinMode for the USB Switch enable Pin

  while (!Serial){  // Waiting for serial connection
    delay(5);
  }
  if (!currentSensor.begin()){ // Check if current sensor is recognized
    systemOK = false;
  }
  currentSensor.setShunt(0.01695, 10.0); // Shunt resisor calibrated based on output current sensor
  // Shunt resistantor:0.015 Ohm 	   max current: 10 A

  currentSensor.setAveragingCount(INAaveragingCount);
  currentSensor.setCurrentConversionTime(INAconversionTime);
}

void setUSBSwitch() { // Check USB Switch signal from Adapter-BCI and set swiitch enable signal
  bool adapterBCICommand = digitalRead(adapterBCI_Aux[0]);
  if (adapterBCICommand == HIGH) {digitalWrite(USBSWTCH_RELAY, HIGH);}
  else {digitalWrite(USBSWTCH_RELAY, LOW);}
  return;
}

// struct used to store data related to the measurement of one BCI output
struct signalData {
  bool  squareWave;
  int   pin;
  float frequency;
  float highVoltage;
  float lowVoltage;
  float dutyCycle;
};

// struct used to store data related to the current and voltage measurements (and systemOK)
struct infoData {
  bool systemOK;
  float measuredCurrent;
  float measuredVoltage;
  float measuredTemperature;
};

// Function to measure one BCI output signal on pin#
signalData signalTest(int pin){

  signalData data; // Initialize data struct
  data.pin = pin;
  data.highVoltage = 0;
  data.lowVoltage = 0;
  data.dutyCycle = 0;
  data.frequency = 0;

  unsigned long highTime = 0;
  unsigned long lowTime = 0;
  unsigned int highValue = 0;
  unsigned int lowValue = 0;

  data.squareWave = true; // stays true unless determined to be false in samples
  
  for (int i=0; i < signalSampleSize; i++){
    noInterrupts();                                             // noInterrupts prevents serial com from interfering with measurements
    highTime += pulseIn(digitalPins[pin], HIGH, pulseTimeout);  // Measures the time the signal is HIGH (flank to flank), function is blocking
    interrupts();                                               // Allowing interrupts
    delayMicroseconds(pulseDelay);                              // Small delay to ensure transition of flank
    lowValue += analogRead(analogPins[pin]);                    // Measure LOW voltage after HGIH pulse (must be in LOW pulse)

    noInterrupts();                                             // Repeated for LOW pulse
    lowTime  += pulseIn(digitalPins[pin], LOW, pulseTimeout);
    interrupts();
    delayMicroseconds(pulseDelay);
    highValue += analogRead(analogPins[pin]);                   // Meausre HGIH voltage after LOW pulse (must be in HIGH pulse)

    if (highTime == 0 || lowTime== 0){                          // highTime = 0 when pulseIn() times out after pulseTimeout
      data.squareWave = false;                                  
      break;                                                    // When pulseIn() times out, exit the loop to save time
    }
  }
  if (data.squareWave == false){  // Implement method to record (single) analog reading -> Done
    return data;
  }
  // Calculating averages
  highTime = highTime / signalSampleSize;
  lowTime = lowTime / signalSampleSize;
  highValue = highValue / signalSampleSize;
  lowValue = lowValue / signalSampleSize;
  // Calculate voltages
  data.lowVoltage = (lowValue * analogFactor);
  data.highVoltage = (highValue * analogFactor);

  if (highTime > 0 && lowTime > 0){
    data.squareWave = true;
    unsigned long period = highTime + lowTime;          // full cycle in microseconds
    data.frequency = 1e6 / period;                      // convert period (microseconds) to Hz
    data.dutyCycle = (float(highTime) / period) * 100;  // calculate duty cycle HIGH (%)
    return data;
  }
  else{
    data.squareWave = false;
    return data;
  }
}

infoData currentTest(){
  
  infoData data;
  if (!systemOK == true){
    data.systemOK = false;
    data.measuredCurrent = 0;
    data.measuredVoltage = 0;
    return data;
  }
  else{
    data.systemOK = true;
  }
  float currentSum = 0;
  float voltageSum = 0;

  //for (int i = 0; i < currentSampleSize; i++){
  //  currentSum += currentSensor.readCurrent();
  //}
  data.measuredCurrent = currentSensor.getCurrent_mA();

  //for (int i =0; i < voltageSampleSize; i++){
  //  voltageSum += currentSensor.readBusVoltage();
  //}
  data.measuredVoltage = currentSensor.getBusVoltage_V();
  data.measuredTemperature = currentSensor.readDieTemp();
  return data;
}

// Function to round off floats to save space in JSON message
float roundFloat (float value, int decimals){
  float factor = pow(10, decimals);
  return round(value * factor) / factor;
}

// For debugging:
void printData(signalData data){
  //if (data.squareWave ==  true){
    Serial.print("Pin " + String(data.pin)); 
    Serial.print(":  Frequency: " + String(data.frequency, 1) + "Hz    ");
    Serial.print("High: " + String(data.highVoltage, 2) + " V    ");
    Serial.print("Low: " + String(data.lowVoltage, 2) + " V    ");
    Serial.println("Duty Cycle: " + String(data.dutyCycle, 2) + " %");
  //}
  //else{
  //  Serial.println("No square wave @ pin " + String(data.pin));
  //}
}

// Serializes data of single pin measurement to JSON array
void addPinDataJson (JsonArray array,signalData data){
  JsonObject obj = array.add<JsonObject>();
  obj["Type"] = "Output" + String(data.pin);
  obj["MaxVoltage"] = roundFloat(data.highVoltage, 2);
  obj["MinVoltage"] = roundFloat(data.lowVoltage, 2);
  obj["SquareWave"] = data.squareWave;
  obj["Frequency"] = roundFloat(data.frequency, 2);
  obj["DutyCycle"] = roundFloat(data.dutyCycle, 2);
}

// Serializes data of infoData in JSON array
void addInfoJson (JsonArray array, infoData data){
  JsonObject obj = array.add<JsonObject>();
  obj["Type"] = "Info";
  obj["SystemOk"] = data.systemOK;
  obj["Current"] = roundFloat(data.measuredCurrent, 2);
  obj["Voltage"] = roundFloat(data.measuredVoltage, 2);
  obj["Temp"] = roundFloat(data.measuredTemperature, 2);
}

void loop() {
  // Read data for pin A0
  unsigned long timeBefore = millis();   // debugging
  // Create Json array that will store data corresponding to signal and current test
  JsonDocument doc;                                  // Json document 
  JsonArray jsonMsgArray = doc.to<JsonArray>();      // Create Json array that stores all objects (corresponding to each output)
  //unsigned long timeBefore = micros();
  for(int i = 0; i < BCI_OUTPUT_AMOUNT; i++){             // Cycle through all outputs
    signalData outputData = signalTest(i);
    addPinDataJson(jsonMsgArray, outputData);       // Add data to Json array
    setUSBSwitch();
  }
  infoData info = currentTest();
  addInfoJson(jsonMsgArray, info);                // Add data from InfoData info to JSON array
  serializeJson(jsonMsgArray, Serial);            // Serialize JSON array into string and write over serial
  Serial.println();                               // Add NewLine char at the end of JSON (essential for parsing) [\r\n]
  unsigned long timeAfter = millis();  //debugging
  unsigned long duration = timeAfter - timeBefore; //debugging
}

