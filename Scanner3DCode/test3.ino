/* Arduino Pro Micro Scanner Code (DIY 3D Scanner - Super Make Something Episode 8) - https://youtu.be/-qeD2__yK4c
 * by: Alex - Super Make Something
 * date: January 2nd, 2016
 * license: Creative Commons - Attribution - Non-Commercial.  More information available at: http://creativecommons.org/licenses/by-nc/3.0/
 */

// Includes derivative of "ReadWrite" by David A. Mellis and Tom Igoe available at: https://www.arduino.cc/en/Tutorial/ReadWrite
// Includes derivative of EasyDriver board sample code by Joel Bartlett available at: https://www.sparkfun.com/tutorials/400

/*
 * This code contains the follow functions:
 * - void setup(): initializes Serial port, SD card
 * - void loop(): main loop
 * - double readAnalogSensor(): calculates sensed distances in cm.  Sensed values calculated as an average of noSamples consecutive analogRead() calls to eliminate noise
 * - void writeToSD(): Writes sensed distance in cm to SD card file specified by filename variable
 * - void readFromSD(): Prints out contents of SD card file specified by filename variable to Serial
 */

/* 
 * Pinout:
 * SD card attached to SPI bus as follows:
 * - MOSI - pin 16
 * - MISO - pin 14
 * - CLK - pin 15
 * - CS - pin 10
 * 
 * IR Sensor (SHARP GP2Y0A51SK0F: 2-15cm, 5V) attached to microcontroller as follows:
 * - Sense - A3
 * 
 * Turntable stepper motor driver board:
 * - STEP - pin 2
 * - DIR - pin 3 
 * - MS1 - pin 4
 * - MS2 - pin 5
 * - Enable - pin 6
 * 
 * Z-Axis stepper motor driver board:
 * - STEP - pin 7
 * - DIR - pin 8
 * - MS1 - pin 9
 * - MS2 - pin 18 (A0 on Arduino Pro Micro silkscreen)
 * - ENABLE - pin 19 (A1 on Arduino Pro Micro silkscreen)
 */
#include <Arduino.h>
#include <cstring>
#include <esp_system.h>
// #include <SPI.h>
// #include <SD.h>

// #define File scannerValues; // Objeto para manipular o arquivo no cartão SD.
// #define String filename="scn000.txt"; // Nome do arquivo onde os dados do scanner serão salvos.
// #define int csPin=10; // Pino do chip select para o módulo SD.

#define PIN_SENSOR 34  // Pino de entrada analógica para o sensor IR.
const float MCU_VOLTAGE = 5.0;
const int sensorPin[] = { 25 };
const int AVERAGE_OF = 50;

const double THRESHOLD = 30;

const int PIN_DIR_MZ = 26;   // Pino de direção para o motor de passo Z-axis
const int PIN_STEP_MZ = 27;  // Pino de passo para o motor de passo Z-axis

const int PIN_DIR_MT = 32;   // Pino de direção para o motor de passo da plataforma giratória.
const int PIN_STEP_MT = 33;  // Pino de passo para o motor de passo da plataforma giratória
const int PIN_GPI0 = 0;


const int scan_amount = 40;
const int distance_to_center = 8;
float measured_analog = 0;
float analog = 0;
float distance = 0;
float x = 0.0;  //X, Y and Z coordinate
float y = 0.0;
float z = 0.0;
float angle = 0;
float RADIANS = 0.0;
float z_cm = 0.2;  // Amount of height accumulated with each complete rotation of the threaded rod (cm)
int resolution = 5;
int stepsToRestart = 940;

void IRAM_ATTR handleReset() {
  esp_restart();
}

void setup() {

  //Define stepper pins as digital output pins
  pinMode(PIN_STEP_MT, OUTPUT);
  pinMode(PIN_DIR_MT, OUTPUT);


  pinMode(PIN_STEP_MZ, OUTPUT);
  pinMode(PIN_DIR_MZ, OUTPUT);

  pinMode(PIN_GPI0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_GPI0), handleReset, FALLING);

  // (3.141592 / 180.0) * (360/200)
  RADIANS = 0.03141592653;

  // Open serial communications
  Serial.begin(9600);

  while (true) {
    if (Serial.available() > 0) {
      String str = Serial.readStringUntil('/n');

      // Se a string recebida for "OK", sai do loop
      if (str.equals("OK")) {
        break;
      }
    }
  }

  //Debug to Serial
  Serial.print("Initializing SD card... ");
  if (!SD.begin(csPin))
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization success!");
}

void loop() {
  // int vertDistance=20; //Total desired z-axis travel
  int vertDistance = getVerticalDistance() / resolution;  //Total desired z-axis travel
  int thetaCounts = 200;

  // Scan object
  digitalWrite(PIN_DIR_MZ, HIGH);         // clockwise
  for (int j = 0; j < vertDistance; j++)  //Rotate z axis loop
  {
    delay(500);
    for (int i = 0; i < thetaCounts; i++)  //Rotate theta motor for one revolution, read sensor and store
    {
      rotateMotor(PIN_STEP_MT, 1);  //Rotate theta motor one step
      delay(5);
      getDistance();
      angle = angle + RADIANS;
      Serial.printf("%f", x);
      Serial.print(" ");
      Serial.printf("%f", y);
      Serial.print(" ");
      Serial.printf("%f", z);
      Serial.print(" ");
      Serial.println();
      writeToSD(x, y, z); //Write coordenates to SD
    }
    angle = 0;
    // z += ((float)resolution / 200) * z_cm;
    z += 0.1*z_cm;
    rotateMotor(PIN_STEP_MZ, resolution);  //Move z carriage up one step
    stepsToRestart += resolution;
    delay(1000);
    // writeToSD(9999); //Write dummy value to SD for later parsing
  }

  // Scan complete.  Rotate z-axis back to home and pause.
  digitalWrite(PIN_DIR_MZ, LOW);  // counter-clockwise
  delay(10);
  for (int j = 0; j < vertDistance; j++) {
    rotateMotor(PIN_STEP_MZ, resolution);
    delay(10);
  }

  for (int k = 0; k < 3600; k++)  //Pause for one hour (3600 seconds), i.e. freeze until power off because scan is complete.
  {
    delay(1000);
  }

  //readFromSD(); //Debug - Read from SD
}

void handleResetSequence() {
  digitalWrite(PIN_DIR_MZ, LOW);
  delayMicroseconds(1);
  for (int i = 0; i < stepsToRestart; i++) {
    rotateMotor(PIN_STEP_MZ, 1);
  }
  esp_restart();
}

int getVerticalDistance() {
  // Inicialmente, defina a direção do motor Z para subir
  digitalWrite(PIN_DIR_MZ, HIGH);

  int steps = 0;

  getDistance();
  while (distance > 0) {  // THRESHOLD é o valor que determina se o objeto foi detectado
    rotateMotor(PIN_STEP_MZ, resolution);
    delay(10);
    steps += resolution;
    getDistance();
  }

  delay(1000);
  digitalWrite(PIN_DIR_MZ, LOW);
  delay(10);
  for (int i = 0; i < steps; i++) {
    rotateMotor(PIN_STEP_MZ, 1);
    delay(10);
  }

  // Retorne o número de passos que o motor Z fez
  return steps;
}


void rotateMotor(int pinNo, int steps) {

  for (int i = 0; i < steps; i++) {
    digitalWrite(pinNo, LOW);  //LOW to HIGH changes creates the
    delay(1);
    digitalWrite(pinNo, HIGH);  //"Rising Edge" so that the EasyDriver knows when to step.
    delay(1);
    //delayMicroseconds(500); // Delay so that motor has time to move
    //delay(100); // Delay so that motor has time to move
  }
}

//Function that gets the distance from sensor
void getDistance() {
  for (int aa = 0; aa < scan_amount; aa++) {
    measured_analog = analogRead(PIN_SENSOR);
    delay(2);
    analog = analog + measured_analog;
  }
  distance = analog / scan_amount;                                                                     //Get the mean. Divide the scan by the amount of scans.
  analog = 0;                                                                                          //reset the andlog read total value
  measured_analog = 0;                                                                                 //reset the andlog read value
  distance = mapDouble(distance, 0.0, 1023.0, 0.0, 5.0);                                               //Convert analog pin reading to voltage
  distance = -5.40274 * pow(distance, 3) + 28.4823 * pow(distance, 2) - 49.7115 * distance + 31.3444;  //From datasheet
  distance = distance_to_center - distance;                                                            //the distance d = distance from sensor to center - measured distance

  y = (cos(angle) * distance);
  x = (sin(angle) * distance);

  /*//For debug
   * Serial.print(distance); Serial.print("   "); 
  Serial.print(x); Serial.print("   "); 
  Serial.print(y); Serial.print("   "); 
  Serial.print(z); Serial.print("   "); 
  Serial.print(angle); Serial.println("   "); */
}

//Function that writes the value to the SD card
void write_to_SD(float SDx, float SDy, float SDz)
{
  file_values = SD.open(file, FILE_WRITE);
  if (file_values) 
  {
    file_values.print(SDx); file_values.print(" ");
    file_values.print(SDy); file_values.print(" ");
    file_values.println(SDz);
    file_values.close(); 
  }  
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
