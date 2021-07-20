#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Don't know why, but keep AFMS.getMotor(0). Stuff breaks without it.
Adafruit_DCMotor *M[] = {AFMS.getMotor(0), AFMS.getMotor(1), AFMS.getMotor(2), AFMS.getMotor(3)};

#define pi 3.14159267 
#define tol 50       // Encoder error tolerance (also defines speed reduction when approaching target)
#define mspeed 100   // Motor speed 0-255

// Encoder pins and last states for interupt tracking
byte encoder_pins[3][2] = {{2,3},{4,5},{6,7}};
bool encoder_last[] = {LOW, LOW, LOW};

// Current and Target Position for each of the 3 encoders (~ 0-7000 is a full movement)
int CP[] = {0, 0, 0};
int TP[] = {0, 0, 0};

// Matrix representing pull direction of each cable
float C[3][2] = {{cos(0),sin(0)},
                 {cos(2*pi/3),sin(2*pi/3)},
                 {cos(4*pi/3),sin(4*pi/3)}};

int piPin = A0;    // select the input pin for the potentiometer
int pwmin = 0;  // variable to store the value coming from the sensor
int sum = 0;
int counter = 0;

void setup() {
  Serial.begin(115200);
  AFMS.begin();  // begin Motor Shield communication with the default frequency 1.6KHz

  // Setup Encoder pins and pin interrupts
  for (byte k=0;k<3;k++) for (byte i=0;i<2;i++) pinMode(encoder_pins[k][i], INPUT);
  PCICR |=  B00000100;
  PCMSK2 |= B01010100;

  pinMode(piPin, INPUT);

  Serial.println("Type Command--> deg weight:");
}

void loop() {
  if (Serial.available() > 4) {
    Serial.println("=====================");
    int deg = Serial.parseInt();
    int weight = Serial.parseInt();
    Serial.print(deg);
    Serial.print(", ");
    Serial.println(weight);
    
    set_Pos(deg, weight);
  }
  
  Drive_Motors_encoded();
}

void set_Pos(float deg, float w) {
  float theta = deg*pi/180;

  byte n, m;
  if (theta>=0 && theta<2*pi/3) n=0,m=1;
  else if (theta>=2*pi/3 && theta<4*pi/3) n=1,m=2;
  else if (theta>=4*pi/3 && theta<2*pi) n=2,m=0;
  else Serial.println("Input angle out of bounds");

  //  float a = C[n][0];
  //  float b = C[m][0];
  //  float c = C[n][1];
  //  float d = C[m][1];
    
  float det = 1/(C[n][0]*C[m][1]-C[m][0]*C[n][1]);
  float inv[2][2] = {{det*C[m][1], det*-C[m][0]},
                     {det*-C[n][1], det*C[n][0]}};

  TP[n] = inv[0][0]*(w*cos(theta)) + inv[0][1]*(w*sin(theta));
  TP[m] = inv[1][0]*(w*cos(theta)) + inv[1][1]*(w*sin(theta));
  TP[3-(n+m)] = 0;
  
  Serial.print(TP[0]);Serial.print(" ");
  Serial.print(TP[1]);Serial.print(" ");
  Serial.print(TP[2]);Serial.print(" ");
  Serial.println();
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
