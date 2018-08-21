/*
Drone Motor Analysis Tool

  controller.ino

  Arduino script for reading various sensors and controlling the ESC

  2 -> HX711 CLK
  3 -> DOUT
  5V -> VCC
  GND -> GND
*/

#include "FastServo.h"
#include "HX711.h"  //You must have this library in your arduino library folder
 
#define DOUT_PIN          3
#define CLK_PIN           2
#define ESC_PIN           9
#define CURRENT_PIN       A0
#define VOLTAGE_PIN       A2
#define NUM_SAMPLES       20

#define ESC_OFF           0.0f
#define TEST_OFF          0
#define TEST_STEP         1
#define TEST_LIN          2
 
HX711 scale(DOUT_PIN, CLK_PIN);
FastServo esc;

//SENSOR CALIBRATION VALUES
//=================================================================================
float voltage_cal = 46.0f;
float current_cal = 5.0f;
float force_cal = -805.0f; //-805000 for my 3Kg max scale setup (kg) or -805.0 (g)
//=================================================================================

unsigned long lastTime = 0;
unsigned int analogReadings[NUM_SAMPLES];
unsigned int analogIndex = 0;
unsigned long analogTotal = 0;
int test = TEST_OFF;
float dTime = 0.0f;
float elapsedTime = 0.0f;
float current = 0.0f;
float voltage = 0.0f;
float voltage_offset = 0.0f;
float current_offset = 102.0f;
float throttle = ESC_OFF;
float testIdle = 0.0f;
float testRun = 0.0f;
float testStep = 0.0f;
String inputString = "";         // a String to hold incoming data
boolean isParsed = false;  // whether the string is complete
 
//=============================================================================================
// SETUP
//=============================================================================================
void setup() {
  pinMode(CURRENT_PIN, INPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  esc.attach(ESC_PIN,1000,2000);
  esc.write(ESC_OFF);
  Serial.begin(250000);
  inputString.reserve(200);
  scale.set_scale(force_cal);
  scale.tare(); //Reset the scale to 0
  for (int i = 0; i < NUM_SAMPLES; i++) {
    analogReadings[i] = 0;
  }
}
 
//=============================================================================================
// LOOP
//=============================================================================================
void loop() {
  dTime = (float) (micros() - lastTime) / 1000000;
  elapsedTime += dTime;
  lastTime = micros();
  analogTotal -= analogReadings[analogIndex];
  analogReadings[analogIndex] = analogRead(CURRENT_PIN);
  analogTotal += analogReadings[analogIndex];
  analogIndex += 1;
  if (analogIndex >= NUM_SAMPLES) analogIndex = 0;
  current = (analogTotal / (current_cal * NUM_SAMPLES)) - current_offset;
  //current = (analogRead(CURRENT_PIN) / current_cal) - current_offset;
  voltage = (analogRead(VOLTAGE_PIN) / voltage_cal) - voltage_offset;

  if (isParsed) {
    char cmd = inputString[0];
    switch (cmd) {
      case 'p':
        test = TEST_OFF;
        setThrottle(inputString.substring(1).toFloat());
        break;
      case 'i':
        test = TEST_OFF;
        setThrottle(ESC_OFF);
        testIdle = inputString.substring(1).toFloat();
        break;
      case 'r':
        test = TEST_OFF;
        setThrottle(ESC_OFF);
        testRun = inputString.substring(1).toFloat();
        break;
      case 's':
        testStep = inputString.substring(1).toFloat();
        elapsedTime = 0.0f;
        test = TEST_STEP;
        break;
      case 'l':
        testStep = inputString.substring(1).toFloat();
        elapsedTime = 0.0f;
        test = TEST_LIN;
        break;
      case 'z':
        test = TEST_OFF;
        setThrottle(ESC_OFF);
        scale.tare();
        current_offset += current;
        voltage_offset += (voltage - 0.34f);
        break;
    }
    inputString = "";
    isParsed = false;
  }
  if (test == TEST_STEP) {
    if (elapsedTime < testStep) setThrottle(testIdle);
    else if (elapsedTime < (2 * testStep)) setThrottle(testRun);
    else if (elapsedTime < (3 * testStep)) setThrottle(testIdle);
    else {
      setThrottle(ESC_OFF);
      test = TEST_OFF;
    }
  }
  
  // Example serial string: "Run Time(s): 25.687 / Loop Time(s): 0.01066 / Battery(V): 16.7 / Throttle(%): 0.00 / Thrust(g): 0.0 / Current(A): 0.0 / Test: 0"
  String dataStr = "Run Time(s): " + String(elapsedTime, 5);
  dataStr += " / Loop Time(s): " + String(dTime, 5);
  dataStr += " / Battery(V): " + String(voltage, 1);
  dataStr += " / Throttle(%): " + String(throttle,2);
  dataStr += " / Thrust(g): " + String(scale.get_units(), 1);
  dataStr += " / Current(A): " + String(current,1);
  dataStr += " / Test: " + String(test);
  Serial.println(dataStr);
}

//=============================================================================================
//  Function that writes to the ESC and global variable
//=============================================================================================
void setThrottle(float percent) {
  throttle = percent;
  esc.write(percent);
}

//=============================================================================================
//  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
//  routine is run between each time loop() runs, so using delay inside loop can
//  delay response. Multiple bytes of data may be available.
//=============================================================================================
void serialEvent() {
  while (Serial.available() && (isParsed == false)) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      isParsed = true;
    }
    else inputString += inChar;
  }
}
