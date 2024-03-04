/*	
	Taxi_PID_Testing.ino
	PID used to move robot car 80cm. 
  Maintains acceptable acceleration.
	
	Ben Kessler
*/

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
volatile int counter_A = 0;
volatile int counter_B = 0;

// Interrupt Service Routines:

// Motor A pulse count ISR
void ISR_countA()  
{
  counter_A++;  // increment Motor A counter value
} 

// Motor B pulse count ISR
void ISR_countB()  
{
  counter_B++;  // increment Motor B counter value
}

//Determaining Position / heading:

float target = 80; //Target position
float pos = 0; //Car Positon 
float heading = 0; //Car Heading

void updateEncoderPos(){//update encoder predicted position
  noInterrupts(); // Disable interrupts
  int steps = (counter_A + counter_B)/2; // Copy the value for use
  counter_A = 0; // Reset the counter
  counter_B = 0; // Reset the counter
  interrupts(); // Re-enable interrupts
  pos += steps*heading*cm_step; //update position (in cm) (accounting for heading/direction)
}

//PID:

//Vars for reducing momentum error:
int dir_result = 0; //signal sent to motor controller
int pwr_result = 0; //signal sent to motor controller
int dir_old = dir_result; //store dir
int maxSpeed = 150; //safe speed ~= 39.13 cm/s (real max is 255)
int minSpeed = 30; //where motor stops spinning ~= 9.06 cm/s

//I and D memory
float prevT; //Previous Time
float ePrev; //Previous Error
float eIntegral; //Integrated Error

//PID constants (largest distance is about 80cm)
const float kp = maxSpeed/40; //Proportional (max speed is approximatly 40cm/s so at his speed it will travle 40 cm in 1 second) (it takes 1s to break)
const float ki = 0.05; //Integral (solve for value need to make up for missing distance using double integral)
const float kd = 3.66479/10; //Derivative (3.66479 = units/(cm/s)) (reduce speed by 90% when within 40cm)

//PID function
void PID(){
  //time differance:
  long currT = micros(); //get time stamp (micro seconds)
  float deltaT = ((float) (currT - prevT))/1.0e6; //get time differance
  prevT = currT; //store time

  //error calcs:
  float e = target - pos; //error in position
  eIntegral += e*deltaT; //update approximate integral of error (= eInt + e*dt)
  float de_dt = (e - ePrev)/deltaT; //calculate approximate derivative of error (= de/dt)
  float u = kp*e + ki*eIntegral + kd*de_dt; //control signal u(t)

  //for assesing PID
  Serial.print("Error: ");
  Serial.print(e);
  Serial.print(", Integral: ");
  Serial.print(eIntegral);
  Serial.print(", deltaT: ");
  Serial.print(deltaT);
  Serial.print(", de_dt: ");
  Serial.print(de_dt);
  Serial.print(", PID Output: ");
  Serial.println(u);

  //Motor Direction:
  int dir = 0; //don't move
  if (u > 35){dir = 1;} //move forward
  if (u < -35){dir = -1;} //move backward
  dir_result = dir; //update global

  //Motor Power:
  int pwr = (int) fabs(u); //PWM value signal
  if (pwr > 200){
    pwr = 200; //ensure in range
  }
  pwr_result = pwr; //update gloabal

  //Store Error:
  ePrev = e;
}

//ramp up varible
int ramp = 35; //to reduce inital acceleration and ensure encoder accuracy

void setup() {
  Serial.begin(9600);

  // Attach the Interrupts to their ISR's 
  pinMode(MOTOR_A.encoder, INPUT);
  pinMode(MOTOR_B.encoder, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_A.encoder), ISR_countA, RISING);  // Increase counter A when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(MOTOR_B.encoder), ISR_countB, RISING);  // Increase counter B when speed sensor pin goes High

  //Configure outputs:
  pinMode(MOTOR_A.PWM, OUTPUT);
  pinMode(MOTOR_B.PWM, OUTPUT);
  pinMode(MOTOR_A.i2, OUTPUT);
  pinMode(MOTOR_B.i2, OUTPUT);
  pinMode(MOTOR_A.i1, OUTPUT);
  pinMode(MOTOR_B.i1, OUTPUT);

  prevT = micros(); //initial time
  delay(1000/4); //prevents time from being to short
  ePrev = target; //inital speed/ chang in error should be 0
}

void loop() {
  PID(); //get signals
  if (pwr_result>ramp){
    pwr_result = ramp; //ensures encoder accuracy by reducing initial acceration
    ramp += 30; //takes about 5 time steps (1.25s) to reach max speed.
  }

  //ensure full stops to reduce error of momentum:
  if (dir_result != dir_old){
    if (dir_old == 0){heading = dir_result;} //ensure correct pos updates
    else if (dir_old == -dir_result){dir_result = 0;} //ensure full stop to reduce momentuom error
  }

  //for assesing output
  Serial.print("dir: ");
  Serial.print(dir_result);
  Serial.print(", pwr: ");
  Serial.println(pwr_result);

  translational(dir_result, pwr_result); //move
  dir_old = dir_result; //save dir

  delay(1000/4); //delay 1s (long enough to get a decent estimate of velocity)

  updateEncoderPos(); //updates estimated car position
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
