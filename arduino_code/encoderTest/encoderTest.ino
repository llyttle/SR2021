#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Encoder.h>

Encoder encoder_1(2, 3);
Encoder encoder_2(4, 5);
Encoder encoder_3(6, 7);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Don't know why, but keep AFMS.getMotor(0). Stuff breaks without it.
Adafruit_DCMotor *M[] = {AFMS.getMotor(0), AFMS.getMotor(1), AFMS.getMotor(2), AFMS.getMotor(3)};

#define D 600000 // Semi-arbitrary number representing the distance each cable is wound

float mspeed = 50;   // Speed 0-255

float wind_time = 4000;
float unwind_time = 3600;
float pause_time = 500;

long start = -1*(wind_time + unwind_time + pause_time + 1);
long elapsed;

int i = 2;

void setup() {
  Serial.begin(115200);

  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  M[1]->setSpeed(mspeed);
  M[2]->setSpeed(mspeed);
  M[3]->setSpeed(mspeed);

  Serial.println(wind_time/1000);
  Serial.println("Ready for input");
}

long oldPosition_1  = -999;
long oldPosition_2  = -999;
long oldPosition_3  = -999;

void loop() {
  while(!Serial) return;
  while(read_encoder()) {
    if (Serial.available() > 0) {
      Serial.println(Serial.readString());
      start = millis();
    }
    
    Drive_Motors();
  }
}

bool read_encoder() {
  long newPosition_1 = encoder_1.read();
  long newPosition_2 = encoder_2.read();
  long newPosition_3 = encoder_3.read();
  
  if (newPosition_1 != oldPosition_1 || newPosition_2 != oldPosition_2 || newPosition_3 != oldPosition_3) {
    oldPosition_1 = newPosition_1;
    oldPosition_2 = newPosition_2;
    oldPosition_3 = newPosition_3;
  }

  if (millis()%100 == 0) {
    Serial.print(newPosition_1);
    Serial.print(",");
    Serial.print(newPosition_2);
    Serial.print(",");
    Serial.println(newPosition_3);
  }
  
  return true;
}

void Drive_Motors() {
  elapsed = millis()-start;

  if (elapsed < wind_time) M[i]->run(FORWARD);
  else if (elapsed < wind_time + pause_time) M[i]->run(RELEASE);
  else if (elapsed < wind_time+pause_time+unwind_time) M[i]->run(BACKWARD);
  else M[i]->run(RELEASE);
}
