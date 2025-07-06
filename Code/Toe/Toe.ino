/*
   Description: This program is used to control Toe: a bipedal walking robot

        Wiring: The required components are 4x 9g servo motors, an ultrasonic distance sensor, a NRF24L01 radio module, and a Seeed XIAO microcontroller
        The servos are connected with brown wire to GND, red wire to Vin, and orange wire to D0, D1, D2, and D3
        The ultrasonic distance sensor is connected Vcc to 3V3, Trig to D5, Echo to D4, Gnd To GND
        The NRF24L01 is connected 3.3V to 3V3, GND to GND, CSN to D46, MOSI to D51, CE to D48, SCK to D52, MISO to D50
*/

#include "Servo.h"
Servo servo[4];

#include "Ramp.h"
ramp angle[4];

#include "RF24.h"
RF24 radio(6, 7);
const byte chan[6] = "00007";
byte data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte but[][4] = {{2, 3, 6, 7}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

byte cnt = 0;
int yDir = 0;
int xDir = 0;
int duration = 0;                                                                                   //Duration of move from one shape to the next

unsigned long millisPrev = 0;

const int neutral[4] = {98, 88, 99, 87};                                                            //Calibration array defining the neutral positions of each servo

const int tiptoe[4] = {0, 0, -85, 85};                                                              //Array containing the servo offsets to stand up on toes

const int seq[3][6][4] = {   /*[dir][cnt][servo]*/                                                  //Array containing the servo offsets to cycle through walking gait
  {{0, 0, 30, 30}, {30, 30, 5, -5}, { -30, -30, 5, -5}, {0, 0, 30, 30}, {30, 30, 5, -5}, { -30, -30, 5, -5}},
  {{0, 0, -30, -30}, { -30, -30, 0, 0}, { -30, -30, 30, 30}, {0, 0, 30, 30}, {30, 30, 0, 0}, {30, 30, -30, -30}},
  {{0, 0, -30, -30}, { -30, -30, 5, -5}, {30, 30, 5, -5}, {0, 0, -30, -30}, { -30, -30, 5, -5}, {30, 30, 5, -5}}
};

int trig = 5;                                                                                       //Define I/O pins for Ultrasonic Distance Sensor
int echo = 4;


void setup() {
  Serial.begin(9600);
  Serial.println("Serial Communication Initialized");

  radio.begin();                                                                                    //Begin radio communication
  radio.openReadingPipe(1, chan);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Radio Communication Initialized");

  for (int i = 0; i < 4; i++) {                                                                     //Initialize servos
    servo[i].attach(i);
    angle[i].go(neutral[i]);
    servo[i].write(angle[i].update());
  }
  Serial.println("Servos Attached");

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  delay(250);
  Serial.println("Running");
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));                                                                //Read in radio data if available

    if (data[1] == 0) yDir = 0;                                                                     //Forward and reverse controlled by front to back motion of left joystick
    else if (data[1] > 127) yDir = 1;
    else yDir = -1;

    if (data[4] == 0) xDir = 0;                                                                     //Turning controlled by left to right motion of right joystick
    else if (data[4] > 127) xDir = 1;
    else xDir = -1;

    duration = map(data[9], 0, 255, 500, 100);                                                      //Driving speed controlled by right potentiometer
  }
  debounce();

  if ((millis() - millisPrev) > angle[0].getDuration()) {                                           //If the previous shape change is complete:
    if (yDir != 0) {                                                                                //If a walking is commanded
      if (yDir > 0) {                                                                                 //If forward is commanded increment shape
        if (cnt == 5) cnt = 0;
        else cnt++;
      }
      else {                                                                                          //If backward is commanded decrement shape
        if (cnt == 0) cnt = 5;
        else cnt--;
      }
      for (int i = 0; i < 4; i++) {
        angle[i].go(neutral[i] + seq[xDir + 1][cnt][i], duration);                                    //Send ramp variable to angles of next shape in sequence
      }
    }
    else if (Ping() < 5) {                                                                          //If something is detected in front:
      for (int i = 0; i < 4; i++) {
        angle[i].go(neutral[i] + tiptoe[i], duration);                                                //Go up on toes
      }
    }
    else {                                                                                          //Otherwise go to neutral position
      for (int i = 0; i < 4; i++) {
        angle[i].go(neutral[i], duration);
      }
    }
    millisPrev = millis();
  }
  for (int i = 0; i < 4; i++) {
    servo[i].write(angle[i].update());                                                              //Set servo angles to updated ramp angle value
  }
}

void debounce() {                                                                                   //Causes momentary button inputs from controller to trigger on once when pressed and not for the duration of being pressed
  for (int i = 0; i < 4; i++) {
    if (data[but[0][i]]) {
      if (!but[1][i]) {
        but[2][i] = 1;
      }
      else but[2][i] = 0;
      but[1][i] = 1;
    }
    else {
      but[1][i] = 0;
      but[2][i] = 0;
    }
  }
}

int Ping() {                                                                                        //Function to return distance indicated by sensor in inches
  long duration, inches;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);                                                                         //Sends out pulse
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);                                                                   //Measures how long it take for the pulse to return
  inches = duration / 74 / 2;                                                                       //Calculates how many inches sound would travel in this time and divides by 2 for round trip
  inches = constrain(inches, 0, 120);                                                               //Limits readouts to be from 0 to 120 inches
  return inches;                                                                                    //Returns the distance in inches
}
