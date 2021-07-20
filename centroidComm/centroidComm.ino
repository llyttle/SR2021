#include <Servo.h>
Servo servo;

int piPin = A0;    // select the input pin for the potentiometer
int pwmin = 0;  // variable to store the value coming from the sensor
int sum = 0;
int counter = 0;
int currentPos = 90;
int ss = 1000;    //Sensitivity Scalar

void setup() {
  pinMode(piPin, INPUT);
  Serial.begin(115200);

  servo.attach(10);
  servo.write(currentPos);
  delay(100);
}

void loop() {
  pwmin = pulseIn(piPin, HIGH)/200;   //map the pulse to 0 - 100%
  
  if (pwmin != 0) {
    sum += pwmin;
    counter += 1;
  }
  
  if (pwmin == 0 && sum > 0) {
    moveServo(sum/counter);
    sum = 0;
    counter = 0;
  }
}

void moveServo(float avg) {
  float diff = 50.0-avg;
//  float theta = pow(diff, 3)*(31.0/pow(50, 3));   // should map the pulse to 0-62.2 degrees
  float theta = 31 * (pow(diff,3)+(diff*ss)) / (pow(50,3)+(50*ss));
  if (abs(theta) > 2) {
    currentPos += theta;
    servo.write(currentPos);
  }
  
  Serial.println(theta);
}
