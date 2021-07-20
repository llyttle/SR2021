#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Set each of the 3 motor objects
Adafruit_DCMotor *M0 = AFMS.getMotor(1);
Adafruit_DCMotor *M1 = AFMS.getMotor(2);
Adafruit_DCMotor *M2 = AFMS.getMotor(3);

#define pi 3.14159267           // Pi
#define Etol 100                // Encoder error tolerance
#define mspeed 255*1.15         // Motor max speed when driving (0-255)
#define RS 300000               // Motor ramp scalar (higher values = more gradual)
#define Atol  pi/8              // Angular tolerance between object and current position
#define angularStepSize pi/24   // Step size to slow down how often robot needs to update position

// Object and Target Position on PiCam in radians
float op_rad = 0;               // Input from raspberry pi and new_op function
float tp_rad = 0;               // Calculated by set_tp function

int last_w = 0;

// Encoder pin Last Positions;
bool LP0=LOW, LP1=LOW, LP2=LOW;
// Current and Target Position for each of the 3 encoders (~ 0-7000 is a full movement)
int CP0=0, CP1=0, CP2=0;        // Direct encoder readings
int TP0=0, TP1=0, TP2=0;        // Calculated by set_Pos function

// Matrix representing pull direction of each cable
float C[3][2] = {{cos(0),sin(0)},               // M0
                 {cos(2*pi/3),sin(2*pi/3)},     // M1
                 {cos(4*pi/3),sin(4*pi/3)}};    // M2

bool keyboard_ok = false;       // Trigger to start program upon any keyboard input

void setup() {
  Serial.begin(115200);
  
  // begin Motor Shield communication with the default frequency 1.6KHz
  AFMS.begin();

  // Setup Encoder pins and pin interrupts
  pinMode(2, INPUT);pinMode(3, INPUT);pinMode(4, INPUT);pinMode(5, INPUT);pinMode(6, INPUT);pinMode(7, INPUT);
  PCICR |=  B00000100;
  PCMSK2 |= B01010100;

  // Setup Raspberry pi PWM input pin
  pinMode(A0, INPUT);

  Serial.println();
  Serial.println(F("Press ENTER to move Robot"));
}

// Main Loop =================================================================================
void loop() {
  if (Serial.available() != 0) {
    char dumb = Serial.read(); // Flush Serial buffer
    keyboard_ok = true;
  }
  
  if (!Driving_Motors_encoded() && keyboard_ok) { // Only move if motors aren't currently driving
    int weight = 4000;
    if (!new_op()) weight = 0;                    // If no object, point straight up
    else if (last_w == 0) tp_rad = op_rad;        // If object and already vertical, point directly in object direction
    else inc_tp();                                // Otherwise (already looking) only increment tp                                
      
    set_Pos(tp_rad, weight);
//    keyboard_ok = false;
  }
}
// Main Loop =================================================================================

void inc_tp() {
  float diff = abs(tp_rad-op_rad);

  if ((tp_rad+Atol>op_rad && diff>=pi) || (tp_rad-Atol<op_rad && diff<=pi)) {
    tp_rad += angularStepSize;
  }
  else if ((tp_rad+Atol>op_rad && diff<=pi) || (tp_rad-Atol<op_rad && diff>=pi)) {
    tp_rad -= angularStepSize;
  }

  if (tp_rad < 0) tp_rad += 2*pi;
  else if (tp_rad > 2*pi) tp_rad -= 2*pi;
}

bool new_op() {
  int dc = pulseIn(A0, HIGH);   // Ranges from 0 to 20,000

  if (dc != 0) {
      op_rad = dc * ((2*pi)/20000.0);  // Mapping on scale of 0 - 2pi
      op_rad -= (2.0/3.0)*pi;          // rotate camera axis to fit robot axis
      if (op_rad < 0) op_rad += 2*pi;
      return true;
    }
  else return false;
}

void set_Pos(float theta, float w) {
  byte n, m;
  int TP[] = {0, 0, 0};
  if (theta>=0 && theta<2*pi/3) n=0,m=1;
  else if (theta>=2*pi/3 && theta<4*pi/3) n=1,m=2;
  else if (theta>=4*pi/3 && theta<=2*pi) n=2,m=0;
  else Serial.println(F("Input angle out of bounds"));  
    
  float det = 1/(C[n][0]*C[m][1]-C[m][0]*C[n][1]);
  float inv[2][2] = {{det*C[m][1], det*-C[m][0]},
                     {det*-C[n][1], det*C[n][0]}};

  TP[n] = inv[0][0]*(w*cos(theta)) + inv[0][1]*(w*sin(theta));
  TP[m] = inv[1][0]*(w*cos(theta)) + inv[1][1]*(w*sin(theta));
  TP[3-(n+m)] = 0;

  TP0=TP[0], TP1=TP[1], TP2=TP[2];

  last_w = w;

  Serial.print(F("Moving to: "));
  Serial.print(theta*(180/pi));
  Serial.print(F(", "));
  Serial.println(w);
}

bool Driving_Motors_encoded() {
  if ((CP0<TP0+Etol&&CP0>TP0-Etol) && (CP1<TP1+Etol&&CP1>TP1-Etol) && (CP2<TP2+Etol&&CP2>TP2-Etol)) {
    return false;
  }
  else {
    M0->setSpeed(ramp(TP0, CP0));
    if (CP0<TP0-Etol) M0->run(FORWARD);
    else if (CP0>TP0+Etol) M0->run(BACKWARD);
    else M0->run(RELEASE);

    M1->setSpeed(ramp(TP1, CP1));
    if (CP1<TP1-Etol) M1->run(FORWARD);
    else if (CP1>TP1+Etol) M1->run(BACKWARD);
    else M1->run(RELEASE);

    M2->setSpeed(ramp(TP2, CP2));
    if (CP2<TP2-Etol) M2->run(FORWARD);
    else if (CP2>TP2+Etol) M2->run(BACKWARD);
    else M2->run(RELEASE);
    return true;
  }
}

float ramp(float t, float c) {
  return (mspeed*abs(t-c)) / (abs(t-c)+(RS/mspeed));
}

// Encoder Function
ISR (PCINT2_vect) {
  if (digitalRead(2) != LP0) CP0 += (digitalRead(2) == digitalRead(3))?-1:1;
  LP0 = digitalRead(2);

  if (digitalRead(4) != LP1) CP1 += (digitalRead(4) == digitalRead(5))?-1:1;
  LP1 = digitalRead(4);

  if (digitalRead(6) != LP2) CP2 += (digitalRead(6) == digitalRead(7))?-1:1;
  LP2 = digitalRead(6);
}
