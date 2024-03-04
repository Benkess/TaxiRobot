/*	
	DualMotorEncoderDemo.ino
	use of Hardware Interrupts to measure speed from two motors encoders
  works with bluetooth on serial
	
	Ben Kessler
*/

// Include the TimerOne Library from Paul Stoffregen
#include "TimerOne.h"

// Constants for Interrupt Pins
// Change values if not using Arduino Uno

// Motor Pins
typedef struct MotorPinsStruct {
	const byte encoder; //Interrupt Pin
	const int PWM; //Motor Driver PWM
  const int i2; //Motor Driver i2
  const int i1; //Motor Driver i1
} MotorPins;

MotorPins MOTOR_A = {3,11,9,8};  // Motor 2 - INT 1 - Left Motor - CCW = Forward
MotorPins MOTOR_B = {2,5,6,7};  // Motor 1 - INT 0 - Right Motor - CW = Forward

// Constant for steps in disk
const float stepcount = 20.00;  // 20 Slots in disk, change if different

// Constant for wheel diameter
const float wheeldiameter = 59; // Wheel diameter in millimeters, change if different

// Constant for cm/step
const float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
const float cm_step = circumference / stepcount;  // CM per Step

// Integers for pulse counters
unsigned int counter1 = 0;
unsigned int counter2 = 0;

// Interrupt Service Routines

// Motor 1 pulse count ISR
void ISR_count1()  
{
  counter1++;  // increment Motor 1 counter value
} 

// Motor 2 pulse count ISR
void ISR_count2()  
{
  counter2++;  // increment Motor 2 counter value
} 

// TimerOne ISR
void ISR_timerone()
{
  Timer1.detachInterrupt();  // Stop the timer
  Serial.print("Motor Speed 1: "); 
  float rotation1 = counter1*cm_step;  // calculate CM/S for Motor 1
  Serial.print(rotation1);  
  Serial.print(" cm/s, "); 
  Serial.print(counter1);
  Serial.print(" steps - "); 
  counter1 = 0;  //  reset counter to zero
  Serial.print("Motor Speed 2: "); 
  float rotation2 = counter2*cm_step;  // calculate CM/S for Motor 2
  Serial.print(rotation2);  
  Serial.print(" cm/s, "); 
  Serial.print(counter2);
  Serial.println(" steps."); 
  counter2 = 0;  //  reset counter to zero
  Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
}

// Define variables to hold the incoming data
String inputString = "";    // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() 
{
  Serial.begin(9600);
  
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt (MOTOR_B.encoder), ISR_count1, RISING);  // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR_A.encoder), ISR_count2, RISING);  // Increase counter 2 when speed sensor pin goes High
  Timer1.attachInterrupt( ISR_timerone ); // Enable the timer
  inputString.reserve(200); // reserve 200 bytes for the inputString:

  pinMode(MOTOR_A.PWM, OUTPUT);
  pinMode(MOTOR_B.PWM, OUTPUT);
  pinMode(MOTOR_A.i2, OUTPUT);
  pinMode(MOTOR_B.i2, OUTPUT);
  pinMode(MOTOR_A.i1, OUTPUT);
  pinMode(MOTOR_B.i1, OUTPUT);

  translational(0,0);
} 

void loop()
{
  if (stringComplete) {
    // Convert the collected string to an integer
    int number = inputString.toInt();

    // Ensure the number is within the desired range
    if (number >= 0 && number <= 255) {
      Serial.print("New motor speed: ");
      Serial.println(number);
      translational(1,number);
    } else {
      Serial.println("Number out of range. Please enter a number between 0 and 255.");
    }

    // Clear the inputString for the next round of input
    inputString = "";
    stringComplete = false; // Reset the flag
  }
}

void setMotor(int dir, int pwmVal, MotorPins motor) {
  analogWrite(motor.PWM, pwmVal); //set speed via PWM
  if (dir==1){//CCW
    digitalWrite(motor.i1,LOW);
    digitalWrite(motor.i2,HIGH);
  }else if (dir==-1){//CW
    digitalWrite(motor.i1,HIGH);
    digitalWrite(motor.i2,LOW);
  }else {//stop
    digitalWrite(motor.i1,LOW);
    digitalWrite(motor.i2,LOW);
  }
}

void translational(int dir, int pwmVal){//set motors at same speed and direction
  setMotor(dir, pwmVal, MOTOR_A); //Set Left Motor
  setMotor(-dir, pwmVal, MOTOR_B); //Set Right Motor (oppisite rotation)
}

void rotational(int dir, int pwmVal){//set motors at same speed, oppisite direction
  setMotor(dir, pwmVal, MOTOR_A); //Set Left Motor
  setMotor(dir, pwmVal, MOTOR_B); //Set Right Motor (same rotation)
}

void serialEvent() {
  while (Serial.available()) {
    // Get the new byte:
    char inChar = (char)Serial.read();
    // Add it to the inputString:
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
