/*
 * Ngima Hyolmo (Nima)
 * NEET AM
 * Intro to Autonomous Machines
 */


//DC MOTOR, MOTOR SHIELD and SENSOR SHIELD:

#include <DRV8835MotorShield.h>
DRV8835MotorShield motors;
int leftspeed = 80; 
int rightspeed = 95;



//Servo Motor:
#include <Servo.h>
Servo myservo;
#define servo_pin 46

const int offset = 6;
int servoPosition = 96;    // variable to store the servo position
int y = 3;



//To drive straight, motors need to have slightly different speed because of assymetric bot 
int v1 = 160;
int v2 = 135;
float k = 30.0;

int pin_arr[5] = {A1,A2,A3,A4,A5};

const float maxim = 990.0;
const float minim = 150.0;



void setup() {
  Serial.begin(9600);
  myservo.attach(servo_pin);
  myservo.write(offset+90);

  
  for (int i = 0; i<5; i++) {
    pinMode(pin_arr[i],INPUT);
  }
}




void loop () {

  float y_arr[5];

  
  float y_prime = 0.0;
  float sum = 0.0;
  for (int i = 0; i<5; i++) {
    
    float yi = ((float)analogRead(pin_arr[i]) - minim)/(maxim-minim); //scaling between 0 and 1
    y_arr[i] = yi;
 
    sum += yi;
   
    y_prime += (1- yi)*i;
    
  }
  
  
  
  
  y_prime = y_prime/(5 - sum) + 1.0 ;
  Serial.println(y_prime);
  Serial.println();

  float error = y - y_prime;
  float del = k*error;
  motors.setM1Speed(rightspeed -del);
  motors.setM2Speed(leftspeed  +del);
    
}
