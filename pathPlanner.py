#include <SharpIR.h>
#include <Servo.h>
#include <Wire.h>
#include "FastIMU.h"
#define IR A0 // define signal pin
#define model 1080 // used 1080
#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration


//gyro
//timers
double timer = 0;
float timeStep = 0.01;

//button
const int buttonPin = 2;
int buttonState= 0;

//motors
int pwmA = 11;
int pwmB = 10;
Servo servoA;
Servo servoB;

//dis sensor
SharpIR SharpIR(IR, model);
int dis=0;

// Pitch, Roll and Yaw values
float pitch = 0;
bool platFound=0;
float roll = 0;
float yaw = 0;

//wall
bool atWallBase=false;
bool climbingUpWall=false;


//IMU
MPU6500 IMU;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;
double X= 0;
int Y=0;
int Z=0;
int readings=0;
double time=0;
double time2=0;
double time3=0;
double time4=0;
double angle=0;
bool poleFound=false;
bool dir=true;
bool sweep=false;
bool finalTurn=false;

void setup() {
  //pinMode(buttonPin, INPUT);

  while (!Serial) {
      ;
    }
  
    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) {
      Serial.print("Error initializing IMU: ");
      Serial.println(err);
      while (true) {
        ;
      }
  }      

  #ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration & data example");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  
  Serial.println("Keep IMU level.");
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
  #endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  pinMode(pwmA, OUTPUT);
  pinMode(pwmB,OUTPUT);
  Serial.begin(115200);
  //Serial.println("Robojax Sharp IR  ");
  servoA.attach(pwmA);
  servoB.attach(pwmB);
}


void drive(int ang){

  bool right = true;
  if (angle >= ang && right == true){ //drifting to the right
    servoA.write(1435);
    servoB.write(1565);
    right = false;
  }
  else if(angle <= -1 * ang && right == false){
    servoA.write(1455);
    servoB.write(2000);
    right = true;
  }
  
  /*
  if (abs(angle) < ang){
    servoA.write(1435);
    servoB.write(2000);
  }
  else if(angle > ang){ //drifting to the right
    servoA.write(1435);
    servoB.write(1565);
    }

  else if(angle < -1 * ang){ //drifting left
   servoA.write(1455);
   servoB.write(2000);
  }
  */
}
void stop(int time){
  servoA.writeMicroseconds(1500);
  servoB.writeMicroseconds(1500);  
  delay(time);
}

void turnLeft(int ang){
   if(dis < 25){
    stop(2000);
    time=0;
    time2=0;
    X=0;
    readings=0;
    poleFound=true;
   }  
  
   if (abs(angle) < ang * 85/90){
    servoA.write(1490);
    servoB.write(1490);
    //Serial.print(" TEST Angle = ");
    //Serial.println(angle);  
  }
  else{
    //Serial.print("Stop Angle = ");
    //Serial.println(angle);  
    stop(2000);
    time=0;
    time2=0;
    X=0;
    readings=0;
    dir=!dir;
    if(sweep) {
      finalTurn = true;
    }
    }
}

void turnRight(int ang){
  if(dis < 25){
      stop(2000);
      time=0;
      time2=0;
      X=0;
      readings=0;
      poleFound=true;
   }  
  if (abs(angle) < ang * 85/90){
    servoA.writeMicroseconds(1515);
    servoB.writeMicroseconds(1515);
    //Serial.print(" TEST Angle = ");
    //Serial.println(angle);  
  }
  else{
    //Serial.print("Stop Angle = ");
    //Serial.println(angle);  
    stop(2000);
    time=0;
    time2=0;
    X=0;
    readings=0;
    dir=!dir;
    sweep=!sweep;
    }
}

void receiveGyro(){
  IMU.update();
  IMU.getGyro(&gyroData);
  //Serial.print("X = ");
  //Serial.println(gyroData.gyroX);
  /*Serial.print("y = ");
  Serial.println(gyroData.gyroY);
  Serial.print("z = ");
  Serial.println(gyroData.gyroZ);*/
  X+= gyroData.gyroX;
  //Serial.print("sum X = ");
  //Serial.println(X);
  readings++;
  if (time == 0){
    time = millis();
  }
  double Vavg = X/readings;
  time2 = millis();
  double newtime = (time2 - time)/1000.0;
  //Serial.print("Diff = ");
  //Serial.println(newtime);
  angle=newtime*Vavg; 
   
}
void getOverWall() {
  /*
  boolean downFromWall = false; 
  receiveGyro();
  Serial.print("bool = ");
  Serial.println(downFromWall);
  Serial.print("AccelZ = ");
  Serial.print((accelData.accelZ + calib.accelBias[2])*1000*9.81);
  Serial.println();
  Serial.print("AccelX = ");
  Serial.print((accelData.accelX + calib.accelBias[0])*1000*9.81);
  Serial.println();   
  drive(5);
  if((accelData.accelZ + calib.accelBias[2])*1000*9.81 < -9.0*1000){
    downFromWall=true;
  }
  if(downFromWall=true && (accelData.accelX + calib.accelBias[0])*1000*9.81 < -9.0*1000){
    //delay(500);
    //stop(2000);
  }
  //delay(500);
  */
  drive(2); 
  //Serial.print("Dist: ");
  //Serial.println(dis);
  if(dis<30){
    atWallBase=true;
  }
  if(atWallBase==true && dis > 80){
    climbingUpWall=true;    
  }
  if(climbingUpWall == true && dis < 30){
    delay(5000);
    stop(2000);
  }
        
}

void findPole(){
int counter=0;
bool atPole = false;
  while(counter<3){
  
   receiveGyro();
   dis=SharpIR.distance();
   if(poleFound){
    if(dis>20 && !atPole){
    servoA.writeMicroseconds(1200);
    servoB.writeMicroseconds(1800);
    
    }
    if(dis <= 15) {
      atPole = true;
      servoA.writeMicroseconds(1000);
      servoB.writeMicroseconds(2000);
      delay(750);
      servoA.writeMicroseconds(1500);
      servoB.writeMicroseconds(1500);
      stop(25);
      while(1){}     
    
    }
  }
  if(dir && !sweep && !finalTurn){
    turnLeft(90);
  }
  else if(!dir && !sweep && !finalTurn){
    turnRight(180);
  }
  
  else if(dir && sweep && !finalTurn){
    turnLeft(90);
    }     
  //Serial.println("finished right turn");
  
  if(!poleFound && sweep && finalTurn){
    servoA.writeMicroseconds(1000);
    servoB.writeMicroseconds(2000);
    delay(2000);
    // stopFunction();
    // delay(1000);
    counter++;
    sweep=false;
    finalTurn = false;
    dir=true;
    }
  }
  while(1){}
}
  
  /*
  while((accelData.accelX + calib.accelBias[0]) > 5){
    receiveGyro();
    drive(5);
  }
  */


void loop() {
  dis=SharpIR.distance();
  receiveGyro();
  //dis=SharpIR.distance();
  // turnLeft(90);
  //drive(2);
  //getOverWall();
  findPole();
  //turnLeft(90);
  //while(1){}
}
