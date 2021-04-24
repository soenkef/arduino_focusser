
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

// temperature
int tempSensorValue = 0;
float temperature, humedity;
int tempIntervall = 500;

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

int stepCount = 0;  // number of steps the motor has taken

void setup() {
  // nothing to do inside the setup
  dht.begin(); //DHT11 Sensor starten
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
  Serial.print("Temp: ");
  Serial.println(temperature);
  
  // read the sensor value of poti:
  sensorReading = analogRead(POTI_PIN);
  Serial.print("Poti: ");
  Serial.print(sensorReading);
  Serial.print(" - ");
 
  // map it to a range from 0 to 100:
  motorSpeed = map(sensorReading, 0, 1023, 0, 100);
  Serial.println(motorSpeed);
  voltage = sensorReading * (5.0 / 1023.0);
  // print out the value you read:
  Serial.print("Volt: ");
  Serial.println(voltage);

  
  Serial.println("------------------------------------");
  delay(tempIntervall);
  
  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.setCursor(0, 0);;
  display.print(F("Temp:    "));
  display.print(temperature);
  display.print(F(" "));
  display.print(char(247)); // Grad-Zeichen
  display.println(F("C"));
  display.setCursor(0, 10);
  display.print(F("Feuchte: "));
  display.print(humedity);
  display.println(F(" %"));
  display.setCursor(0, 20);
  display.print(F("Speed: "));
  display.print(motorSpeed);
  display.println(F(" %"));
  display.display();  
  
  // set the motor speed:
  if (motorSpeed > 0) {
    myStepper.setSpeed(motorSpeed);
    // step 1/100 of a revolution:
    myStepper.step(stepsPerRevolution / 100);
  }
}
