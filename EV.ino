#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 13 // WHITE
#define PWM 9
#define IN2 5
#define IN1 4
#define buttonPin 3

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile bool startLoop = false;

//PID Variables
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int target = 3996;

//PID Constants

float kp = .2;
float kd = 0.0;
float ki = 0;




void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), buttonPressed, FALLING);
  
}

void loop() {
  if (startLoop) {
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    int pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      pos = posi;
    }

    //error
    int e = pos - target;

    //derivative
    float dedt = (e-eprev)/(deltaT);

    //integral
    eintegral = eintegral + e*deltaT;

    //control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    //motor power
    float pwr = fabs(u);
    if (pwr > 255) {
      pwr = 255;
    }

    //motor direction

    int dir = 1;
    if (u < 0) {
      dir = -1;
    }

    setMotor(dir,pwr,PWM,IN1,IN2);

    eprev = e;


    Serial.print("Control Signal:");
    Serial.print(u);
    Serial.print(", Pos:");

    Serial.println(pos);


  }
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

void buttonPressed() {
  startLoop = true;
}
