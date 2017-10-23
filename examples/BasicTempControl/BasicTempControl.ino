/*
   AutoPID BasicTempControl Example Sketch

   This program reads a dallas temperature probe as input, potentiometer as setpoint, drives an analog output.
   It lights an LED when the temperature has reached the setpoint.
*/
#include <AutoPID.h>
#include <DallasTemperature.h>
#include <OneWire.h>

//pins
#define POT_PIN A0
#define OUTPUT_PIN A1
#define TEMP_PROBE_PIN 5
#define LED_PIN 6

#define TEMP_READ_DELAY 800 //can only read digital temp sensor every ~750ms

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP .12
#define KI .0003
#define KD 0

double temperature, setPoint, outputVal;

OneWire oneWire(TEMP_PROBE_PIN);
DallasTemperature temperatureSensors(&oneWire);

//input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&temperature, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

unsigned long lastTempUpdate; //tracks clock time of last temp update

//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened
bool updateTemperature() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    temperature = temperatureSensors.getTempFByIndex(0); //get temp reading
    lastTempUpdate = millis();
    temperatureSensors.requestTemperatures(); //request reading for next time
    return true;
  }
  return false;
}//void updateTemperature


void setup() {
  pinMode(POT_PIN, INPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  temperatureSensors.begin();
  temperatureSensors.requestTemperatures();
  while (!updateTemperature()) {} //wait until temp sensor updated

  //if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  myPID.setBangBang(4);
  //set PID update interval to 4000ms
  myPID.setTimeStep(4000);

}//void setup


void loop() {
  updateTemperature();
  setPoint = analogRead(POT_PIN);
  myPID.run(); //call every loop, updates automatically at certain time interval
  analogWrite(OUTPUT_PIN, outputVal);
  digitalWrite(LED_PIN, myPID.atSetPoint(1)); //light up LED when we're at setpoint +-1 degree

}//void loop
