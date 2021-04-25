
/*
 Stepper Motor Control - speed control

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.
 A potentiometer is connected to analog input 0.

 The motor will rotate in a clockwise direction. The higher the potentiometer value,
 the faster the motor speed. Because setSpeed() sets the delay between steps,
 you may notice the motor is less responsive to changes in the sensor value at
 low speeds.

 todo:
 - make motor turn clockwise and other direction
 - set two buttons (forward, backward)
 - set one button for display on
 - adapt movement of stepper by poti very sensible
 - temperature

 */
// Display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "DHT.h" //DHT Bibliothek laden

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// DHT
#define DHTPIN 7 //Der Sensor wird an PIN 2 angeschlossen    
#define DHTTYPE DHT11    // Es handelt sich um den DHT11 Sensor
DHT dht(DHTPIN, DHTTYPE); //Der Sensor wird ab jetzt mit „dth“ angesprochen

// stepper
#include <Stepper.h>

// for potentiometer
#define   POTI_PIN          A0
int sensorReading = 0;
int motorSpeed = 0;
float voltage;
int result = 0;
int limitTop = 970;
const byte e = 5; // Nur Änderungen größer e werden bearbeitet.
int last;
int lastval_0;  // für analogRead(A0)
int mySensVals[14] = {1, 3, 5, 7, 9, 10, 15, 20, 25, 30, 40, 55, 75, 95};

#define BUTTON_UP 8
#define BUTTON_DOWN 9
#define BUTTON_EXTRA 10
int buttonUpState = 0;
int buttonDownState = 0;
int buttonExtraState = 0;

// temperature
int tempSensorValue = 0;
float temperature, humedity;
float temperatureNew, humedityNew;
int tempIntervall = 300;

// for your motor
int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 3, 4, 5, 6);

int stepCount = 0;  // number of steps the motor has taken

int filter(int in, int& last) {
   if ( (in < (last - e) || (last + e) < in ) ) {
       last = in;
   }
   return last;
}

void setup() {
  // nothing to do inside the setup
  dht.begin(); //DHT11 Sensor starten
  pinMode(BUTTON_UP, INPUT);
  pinMode(BUTTON_DOWN, INPUT);
  pinMode(BUTTON_EXTRA, INPUT);
  //attachInterrupt (digitalPinToInterrupt (BUTTON_UP), switchPressed, HIGH);  // attach interrupt handler
  //attachInterrupt (digitalPinToInterrupt (BUTTON_DOWN), switchPressed, HIGH);  // attach interrupt handler
  //attachInterrupt (digitalPinToInterrupt (BUTTON_EXTRA), switchPressed, CHANGE);  // attach interrupt handler  
  Serial.begin(9600);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
}

void loop() {

  humedity = dht.readHumidity(); //die Luftfeuchtigkeit auslesen und unter „Luftfeutchtigkeit“ speichern
  temperature = dht.readTemperature();//die Temperatur auslesen und unter „Temperatur“ speichern
  delay(tempIntervall);
  
  // read the sensor value of poti:
  sensorReading = analogRead(POTI_PIN);
  motorSpeed = map(sensorReading, 0, 1023, 0, 1000);

  calculateSteps(motorSpeed);
  
  serialOutput();

  // button input read
  buttonUpState = digitalRead(BUTTON_UP);
  buttonDownState = digitalRead(BUTTON_DOWN);
  buttonExtraState = digitalRead(BUTTON_EXTRA);
  if (buttonUpState == HIGH) {
    Serial.println("ButtonUp");
    // set the motor speed:
    if (motorSpeed > 0) {     
      makeSteps(0);
    }  
  }

  if (buttonDownState == HIGH) {
    Serial.println("ButtonDown");
    // set the motor speed:
    if (motorSpeed > 0) {
      // calculate better!
      makeSteps(1);
    }  
  }
  if (buttonExtraState == HIGH) {
    Serial.println("ButtonExtra");
    display.clearDisplay();
    display.display();
  }
  
  // use display output
  showOnDisplay();
}

void calculateSteps(int potiValue) {
  //last = lastval_0;
   // if (last != filter(analogRead(POTI_PIN), lastval_0)) {
       // map it to a range from 0 to 100:
       //motorSpeed = map(sensorReading, 0, 1023, 0, 1000);
    //   Serial.print("Letzter Wert: ");
      // Serial.println(lastval_0);
       //delay(2000);
   //}
  // todo: do calculation an stepper way in extra function by interrupt
  result = potiValue;
  if (potiValue == 0) {
    motorSpeed = 1;
    result = 1;
  }
  if (potiValue > limitTop) {
    motorSpeed = 1024;
    result = 1024;
  }
  if (potiValue < 200) {
    result = mySensVals[13];
  }
  if (potiValue < 190) {
    result = mySensVals[12];
  }
  if (potiValue < 180) {
    result = mySensVals[11];
  }
  if (potiValue < 170) {
    result = mySensVals[10];
  }
  if (potiValue < 150) {
    result = mySensVals[9];
  }
  if (potiValue < 140) {
    result = mySensVals[8];
  }  
  if (potiValue < 130) {
    result = mySensVals[6];
  }
  if (potiValue < 120) {
    result = mySensVals[5];
  }
  if (potiValue < 100) {
    result = mySensVals[3];
  }
  if (potiValue < 80) {
    result = mySensVals[2];
  }
  if (potiValue < 60) {
    result = mySensVals[1];
  }
  if (potiValue < 40) {
    result = mySensVals[0];
  }
  if (potiValue < 30) {
    result = 1;
  }
  //if (potiValue < limitBottom) {
  //  motorSpeed = 10;
  //  result = 1;
  //}
  
  
}

void showOnDisplay() {
  // display text
  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.setCursor(0, 0);;
  display.print(F("T: "));
  display.print(temperature);
  display.print(char(247)); // Grad-Zeichen
  display.println(F("C"));
  display.setCursor(0, 10);
  display.print(F("F: "));
  display.print(humedity);
  display.println(F("%"));
  display.setCursor(0, 20);
  display.print(F("S: "));
  display.print(result);
  display.println(F(" steps"));
  display.display();  
}

void serialOutput() {
  // todo: do serial output in extra function
  Serial.print("Temp: ");
  Serial.println(temperature);
  Serial.print("Poti: ");
  Serial.print(sensorReading);
  Serial.print(" - ");
  Serial.print("motorspeed: ");
  Serial.println(motorSpeed);
  voltage = sensorReading * (5.0 / 1023.0);
  // print out the value you read:
  Serial.print("Volt: ");
  Serial.println(voltage);
  Serial.println("------------------------------------");
 }

void makeSteps(int direction) {
  if (direction == 1) {
    result = -1 * result;
    Serial.print("negativer Wert!");
  }
  
  Serial.print("SetSpeed: ");
  Serial.println(result);
  myStepper.setSpeed(5);
  // step 1/100 of a revolution:
  //myStepper.step(stepsPerRevolution / motorSpeed);
  myStepper.step(result); // Der Motor macht 2048 Schritte, das entspricht einer Umdrehung..
}

void switchPressed() {
  //buttonUpState != buttonUpState;
  Serial.println("ButtonUpInterrupt");
  int valueOfX = 0;
}
