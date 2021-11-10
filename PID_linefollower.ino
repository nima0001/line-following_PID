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
//To drive straight, motors need to have slightly different speed because of assymetric bot 
int v1 = 160;
int v2 = 135;





//Servo Motor:
#include <Servo.h>
Servo myservo;
#define servo_pin 46

const int offset = 6;
int servoPosition = 96;    // variable to store the servo position








//Line following IR sensor:
int pin_arr[5] = {A1,A2,A3,A4,A5};
const float maxim = 990.0;
const float minim = 150.0;



//PID Variables:
int y = 3; //target
float integral = 0.0;
float previousError = 0.0;


float Kp = 30.0; //Proportional parameter
float Ki = 400.0;  //Integral Parameter
float Kd = 40.0; //Derivative parameter
float delta_t = 100.0; //Time between subsequent change

float t_start;
float t_end;

void setup() {
  Serial.begin(9600);
  myservo.attach(servo_pin);
  myservo.write(offset+90);
  //Line following sensor:
  for (int i = 0; i<5; i++) {
    pinMode(pin_arr[i],INPUT);
  }
  
}




void loop () {
  unsigned long StartTime = micros();

  //Calculating sensor output:
  float y_prime = 0.0; //output from sensor (to be updated)
  float sum = 0.0;
  for (int i = 0; i<5; i++) {  
    float yi = ((float)analogRead(pin_arr[i]) - minim)/(maxim-minim); //scaling between 0 and 1
    
    sum += yi;  
    y_prime += (1- yi)*i;  
  }
    
  y_prime = y_prime/(5 - sum) + 1.0 ;
  Serial.println(y_prime);
  Serial.println();
  
  float error = y - y_prime;




  //PID:
   
  integral += error * delta_t; //defined in global scope
  float derivative = (error - previousError)/delta_t;

  float del = Kp*error + Ki*integral + Kd*derivative;
  previousError = error;



  //MOTOR CONTROL:
  motors.setM1Speed(rightspeed -del);
  motors.setM2Speed(leftspeed  +del);



  //NO CODES DOWN HERE:
  //MAKING SURE time difference is exactly delta_t
  unsigned long CurrentTime = micros();
  unsigned long elapsedTime = ((CurrentTime - StartTime))/1000;
  Serial.println(elapsedTime);
  
  delay(delta_t - elapsedTime); //For the difference in times between two subsequent output to be 'exactly' 1000 milliseconds
   
}
