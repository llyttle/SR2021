#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Don't know why, but keep AFMS.getMotor(0). Stuff breaks without it.
Adafruit_DCMotor *M[] = {AFMS.getMotor(0), AFMS.getMotor(1), AFMS.getMotor(2), AFMS.getMotor(3)};

#define D 600000 // Semi-arbitrary number representing the distance each cable is wound

float mspeed = 150;   // Speed 0-255

float wind_time = D/mspeed;
float unwind_time = wind_time*0.85;

//float wind_time = 100;
//float unwind_time = 0;

int n[] = {2};

void setup() {
  Serial.begin(9600);

  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  M[1]->setSpeed(mspeed);
  M[2]->setSpeed(mspeed);
  M[3]->setSpeed(mspeed);

  Serial.println(wind_time/1000);
  Serial.println("Ready for input");
}

void loop() {
  while (Serial.available() == 0) return;
  Serial.println(Serial.readString());
  Drive_Motors();
}

void Drive_Motors() {
  int NOM = sizeof(n)/sizeof(int);
  int i;

  Serial.println("Driving "+String(NOM)+" Motor(s)");
  for (i=0;i<NOM;i++) M[n[i]]->run(FORWARD);
  delay(wind_time);

  for (i=0;i<NOM;i++) M[n[i]]->run(RELEASE);
  delay(1000);

  for (i=0;i<NOM;i++) M[n[i]]->run(BACKWARD);
  delay(unwind_time);

  for (i=0;i<NOM;i++) M[n[i]]->run(RELEASE);
  Serial.println("Command Finished");
}
