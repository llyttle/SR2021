#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <BasicLinearAlgebra.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Don't know why, but keep AFMS.getMotor(0). Stuff breaks without it.
Adafruit_DCMotor *M[] = {AFMS.getMotor(0), AFMS.getMotor(1), AFMS.getMotor(2), AFMS.getMotor(3)};

using namespace BLA;

#define pi 3.14159267
#define tol 100
#define mspeed 150   // Speed 0-255

long start_time = 0;

byte encoder_pins[3][2] = {{2,3},{4,5},{6,7}};
bool encoder_last[] = {LOW, LOW, LOW};
int CP[] = {0, 0, 0};
int TP[] = {0, 0, 0};

int PosList[] = {-0, -0, 500};

void setup() {
  Serial.begin(115200);

  for (byte k=0;k<3;k++) {
    pinMode(encoder_pins[k][0], INPUT);
    pinMode(encoder_pins[k][1], INPUT);
  }
  PCICR |=  B00000100;
  PCMSK2 |= B01010100;

  AFMS.begin();  // create with the default frequency 1.6KHz
}

void loop() {
//  while(!Serial) return;
  if (Serial.available() != 0) {
    for (byte k=0;k<3;k++) TP[k] += PosList[k];
    Serial.println(Serial.readString());
  }
  
  if (millis()%100 == 0) {
    Serial.print("Encoder1:");Serial.print(CP[0]);Serial.print(" ");
    Serial.print("Encoder2:");Serial.print(CP[1]);Serial.print(" ");
    Serial.print("Encoder3:");Serial.print(CP[2]);Serial.print(" ");
    Serial.println();
  }

  Drive_Motors_encoded();
}

void Drive_Motors_encoded() {
  for (byte k=0;k<3;k++) {
    // Set the speed to start, from 0 (off) to 255 (max speed)
    if (abs(TP[k]-CP[k]) < tol*3) M[k+1]->setSpeed(mspeed/2);
    else M[k+1]->setSpeed(mspeed);
    
    if (CP[k]<TP[k]-tol) M[k+1]->run(FORWARD);
    else if (CP[k]>TP[k]+tol) M[k+1]->run(BACKWARD);
    else M[k+1]->run(RELEASE);
  }
}

// Encoder Function
ISR (PCINT2_vect) {
  for (byte k=0;k<3;k++) {
    if (digitalRead(encoder_pins[k][0]) != encoder_last[k]) {
      CP[k] += (digitalRead(encoder_pins[k][0]) == digitalRead(encoder_pins[k][1]))?-1:1;
    }
    encoder_last[k] = digitalRead(encoder_pins[k][0]);
  }
}
