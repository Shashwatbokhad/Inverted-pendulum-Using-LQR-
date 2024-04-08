#include "Arduino.h"
#include <digitalWriteFast.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>


MPU6050 mpu(Wire);

//Declare NIDEC Motor pins and encoder pins
#define brake  8
#define cw     4
#define rpm    9

#define c_EncoderPinA 2
#define c_EncoderPinB 3

// Declareation of vairiables
float alpha_dot,alphaA,alphai;
float prev_time,current_time,dt;
float k[3] ={-217.11238, -87.33416, -0.098};// K Matrix
float y_setpoint[3] = {0, 0, 0};
float x=0.0;
float x_dt=0.0;
float Ut,Ut_new;
int pwm;
const float pi = 3.14159;

volatile int _EncoderBSet;
volatile long _EncoderTicks = 0;
long copy_EncoderTicks = 0; //new variable for protected copy

/////////////NIDEC Motor//////////////
void nidec_motor_init()
{
  pwm = 255;
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(rpm, OUTPUT);
 
  digitalWrite(brake, HIGH);  //go=1
  digitalWrite(cw, LOW);      //direction ccw
  analogWrite(rpm, pwm);
 
}

void nidec_motor_control(int pwm)
{
  if (pwm < 0)
  {
    digitalWrite(cw, LOW);
    pwm = -pwm;
  }
  else {
    digitalWrite(cw, HIGH);
  }
  pwm = map(pwm,0,255,255,0);
  analogWrite(rpm, pwm);
}

void nidec_motor_brake()
{
  digitalWrite(brake, LOW);  //go=1
  analogWrite(rpm, 255);
}
//////////////////////////////////////



/////////////Timer1 ISR for IMU///////
void timer1_init()
{
    cli(); //Clears the global interrupts
    TIMSK1 = 0x01; //timer5 overflow interrupt enable
    TCCR1B = 0x00; //stop
    TCNT1H = 0xA2; //Counter higher 8 bit value
    TCNT1L = 0x3F; //Counter lower 8 bit value
    TCCR1A = 0x00;
    TCCR1C = 0x00;
    TCCR1B = 0x02; //start Timer, prescaler 8
    sei();   //Enables the global interrupts
}

ISR (TIMER1_OVF_vect)
{
    sei();  
    TCNT1H = 0xA2; //Counter higher 8 bit value
    TCNT1L = 0x3F; //Counter lower 8 bit value
    mpu.update();
    cli();
    x = mpu.getAngleX();
    x_dt = mpu.getGyroX();
    //Serial.println(x);
    //Serial.println(x_dt);
}



void setup()
{
  Serial.begin(9600);
  Wire.begin();
 
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println("MPU begin done!\n");
  Serial.println("Begin Device initialization:\n");
  digitalWrite(c_EncoderPinA, LOW); // turn on pullup resistors
  pinMode(c_EncoderPinB, INPUT); // sets pin B as input
  digitalWrite(c_EncoderPinB, LOW); // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(c_EncoderPinA), HandleMotorInterruptAfirst, RISING);
  nidec_motor_init();
  Serial.println("NIDEC initialized\n");
  timer1_init();
  Serial.println("Timer initialized\n");
  prev_time = millis();
  alpha_dot = 0;
}

void loop()
{


  noInterrupts();
  copy_EncoderTicks = _EncoderTicks;
  interrupts();
 
  delay(30);
  current_time = millis();

  alphaA = copy_EncoderTicks*3.6;
  dt = (current_time - prev_time)/1000;
  alpha_dot = (alphaA - alphai)/dt;
 
  Ut = -1*( k[0]*(x-y_setpoint[0]) + k[1]*(x_dt-y_setpoint[1]) + k[2]*(alpha_dot-y_setpoint[2])); //100;

  // torque to alpha_dot
  //another eq Ut = I*alpha_dot_dot
  // Ut/I = (alpha_dotA - alpha_doti)/dt

  //alpha_dotA = alpha_doti + (Ut/I)*dt;
  //alpha_dotA = constrain(alpha_dotA,-19260,19260);    
  Ut_new = constrain(Ut,-18200,18200);
  ///alpha_dot to desired pwm singnal
 
  pwm = map(Ut_new,-18200,18200,-255,255);

  //alpha_dot = pwm*71.372549;
  alphai = alphaA;
  prev_time = current_time;
  print_data();
 

  /////////////////////////////////        
  nidec_motor_control(pwm) ;
  ////////////////////////////////
 
 
 
}

void print_data(){
  Serial.print("       thetha : ");Serial.print(x);
  Serial.print("       thetha_dot : ");Serial.print(x_dt);
  Serial.print("       alpha_dotA : ");Serial.print(alpha_dot);
  Serial.print("       pwm : ");Serial.println(pwm);

}


void HandleMotorInterruptAfirst()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _EncoderBSet = digitalReadFast(c_EncoderPinB); // read the input pin
 
  // and adjust counter + if A leads B
 if(_EncoderBSet>0) {
  _EncoderTicks++;
 }
 else{
  _EncoderTicks--;
 }

}
