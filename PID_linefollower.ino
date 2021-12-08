/*
 * Ngima Hyolmo (Nima)
 * NEET AM
 * Intro to Autonomous Machines
 */


//DC MOTOR, MOTOR SHIELD and SENSOR SHIELD:

#include <DRV8835MotorShield.h>
DRV8835MotorShield motors;

//Line following IR sensor:
int pin_arr[5] = {A1,A2,A3,A4,A5};
float  y_arr[5]; //for vector valued output from each IR sensors
const float maxim = 990.0;
const float minim = 150.0;

//PID Variables:
int y = 3; //target value 
float integral = 0.0; 
float previousError = 0.0;


float x = 1.5; //with 1.5, 100 and 40 worked best 
int leftspeed = 80.0*x;  
int rightspeed = 95.0*x;


float Kp = 140.0*x; //Proportional parameter 
float Ki = 0.0;     //Integral Parameter
float Kd = 40.0*x;  //Derivative parameter 
float delta_t = 1.5;//Time between subsequent change

float t_start;
float t_end;




//Servo Motor:
#include <Servo.h>
Servo myservo;
#define servo_pin 46

const int offset = 6;
int servoPosition = 96;    // variable to store the servo position




void setup() {
  float t_0 = millis();
  Serial.begin(9600);
  myservo.attach(servo_pin);
  myservo.write(offset+90);
  //Line following sensor:
  for (int i = 0; i<5; i++) {
    pinMode(pin_arr[i],INPUT);
  }
}








void loop () {
  float StartTime = micros();

  //Calculating sensor output:
  float y_prime = 0.0; //output from sensor (to be updated)
  float sum = 0.0;
  for (int i = 0; i<5; i++) {  
    float yi = ((float)analogRead(pin_arr[i]) - minim)/(maxim-minim); //scaling between 0 and 1
    y_arr[i] = yi;
    sum += yi;  
    y_prime += (1- yi)*i;  
  }

 

  //Intersection case:
  if ((y_arr[0] <= 0.2 and y_arr[2] <= 0.2) or (y_arr[4] <= 0.2 and y_arr[2] <= 0.2)) {
    motors.setM1Speed(rightspeed);
    motors.setM2Speed(leftspeed);  
    delay(300); 
  }
  

  //General case:
  else {
    y_prime = y_prime/(5 - sum) + 1.0;
    float error = y - y_prime;  
    
    //PID:
    integral += error * delta_t; //right reimann sum or right rectangle piece-wise approx.
    float derivative = (error - previousError)/delta_t; //forward difference or forward Euler
  
    float del = Kp*error + Ki*integral + Kd*derivative;
    previousError = error;
  
 
  
    //MOTOR CONTROL:
    motors.setM1Speed(rightspeed -del);
    motors.setM2Speed(leftspeed  +del);
  
  
    //NO CODES DOWN HERE:
    //MAKING SURE time difference is exactly delta_t
    float CurrentTime = micros();
    float elapsedTime = ((CurrentTime - StartTime))/1000.0;
    delay(delta_t - elapsedTime); //For the difference in times between two subsequent output to be 'exactly' 1000 milliseconds 
  }
}
