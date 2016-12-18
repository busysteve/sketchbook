// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
#include "linear_regression.h"
int led = 13;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float angX,angY,angZ;

int STD_LOOP_TIME = 19; 
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

float center;
float gcenter;

//********************Change the tuning parameters here**********************
//Setpoint.  Find where the robot is balanced.  
//double Setpoint=-5;
//Point where it switches from conservative to agressive 
int gapDist=180;
//Aggressive
//double aggK=0.5, aggKp=1.0, aggKi=.8, aggKd=.5; 
double aggK=1.5, aggKp=.5, aggKi=.5, aggKd=.5; 
//double aggK=0.5, aggKp=5, aggKi=.5, aggKd=4; 
//Conservative
//double consK=0.2, consKp=.5, consKi=.2, consKd=.2;
double consK=.5, consKp=.2, consKi=.2, consKd=.3;
//***************************************************************************



#define SAMPLES  10

float sample_array[SAMPLES];
float avg_array[SAMPLES];
float lra_array[SAMPLES];
float lrg_array[SAMPLES];
int tlr_array[SAMPLES];
int avg_mod = 0;
int lr_mod = 0;

linear_regression lrx(SAMPLES), lrg(SAMPLES);

void setup(){
  pinMode(led, OUTPUT); 
  pinMode(11, OUTPUT); 
  pinMode(10, OUTPUT); 
  pinMode(6, OUTPUT); 
  pinMode(5, OUTPUT); 
  Wire.begin();
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(24);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(24);
  Wire.endTransmission(true);

  Serial.begin(9600);
  
  delay(1000);
  
  // Blink 2 time for three seconds
  for( int i = 0; i < 2; i++ )
  {
      digitalWrite( led, HIGH );
      delay(100);
      digitalWrite( led, LOW );
      delay(900);
  }
  
  // Blink while sampling
  for( int i = 0; i < SAMPLES; i++ )
  {
      digitalWrite( led, HIGH );
      readChip();
      delay(10);
      digitalWrite( led, LOW );
      delay(90);
      
      int t = millis();
      lrx.log_entry( t, angX );
      lrg.log_entry( t, GyX );
  }
  
  center = 0;
  for( int i = 0; i < SAMPLES; i++ )
  {
      center += avg_array[i];
  }
  center /= SAMPLES;
  
  gcenter = 0;
  for( int i = 0; i < SAMPLES; i++ )
  {
      gcenter += lrg_array[i];
  }
  gcenter /= SAMPLES;
  
  // Blink 2 time for three seconds
  for( int i = 0; i < 2; i++ )
  {
      digitalWrite( led, HIGH );
      delay(100);
      digitalWrite( led, LOW );
      delay(900);
  }


  Serial.println("lraccx,lrgyrx,center,gcenter,angX,avgx,stdx,drive,wait,c_torque");
  
}



void loop(){
  int t = millis();
  readChip();
  
  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(30);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    
  // Averaging
  avg_array[avg_mod%SAMPLES] = angX;
  avg_mod++;
  float avgx = 0;
  float stdx = 0;
  
  // -------
  // average
  // -------
  for( int i = 0; i < SAMPLES; i++ )
  {
     avgx += avg_array[i];
  }
  avgx /= SAMPLES;
  
  // ------------------
  // standard deviation
  // ------------------
  for( int i = 0; i < SAMPLES; i++ )
  {
     stdx += ((avg_array[i] - avgx)*(avg_array[i] - avgx));
  }
  stdx = sqrt(stdx / (SAMPLES-1));
  
  /*
  (8*(C10)-(A10)*(B10))/((8)*(D10)-((A10)^2))
  (B10-(E11*(A10)))/8
  
  m = (SAMPLES*(lrxys)-(ts)*(lrys))/((SAMPLES)*(lrxxs)-((ts*ts))
  b = (lrys-(m*(ts)))/SAMPLES
  */
  
  // ---------------------
  // AcX Linear Regression
  // ---------------------
  tlr_array[lr_mod%SAMPLES] = t;
  lra_array[lr_mod%SAMPLES] = angX;
  lrg_array[lr_mod%SAMPLES] = GyX;
  lr_mod++;

  lrx.log_entry( t, angX );
  lrg.log_entry( t, GyX );
  
  float lraccx = lrx.calc();
  
  float lrgyrx = lrg.calc();  
  
  //lraccx = kalmanCalculate(angX, (GyX/lastLoopUsefulTime), lastLoopTime);
  
#if 1  
  int drive;
  
  double gap = abs(center-lraccx); //distance away from setpoint
  if(gap<gapDist)
    //we're close to setpoint, use conservative tuning parameters
    drive = updatePid(center, lraccx, consK, consKp, consKi, consKd);
  else
    //we're far from setpoint, use aggressive tuning parameters
    drive = updatePid(center, lraccx, aggK, aggKp, aggKi, aggKd);
  
  if(lraccx>(center-170) && lraccx<(center+170))     DriveMotors(drive); 
  else                                                     DriveMotors(0);     // stop motors if situation is hopeless
  
  
  /*
  Serial.print("lraccx = ");
  Serial.print(lraccx);
  Serial.print(" \t| lrgyrx = ");
  Serial.print(lrgyrx);
  Serial.print(" \t| center = ");
  Serial.print(center);
  Serial.print(" \t| angX = ");
  Serial.print(angX);
  Serial.print(" \t| avgx = ");
  Serial.print(avgx);
  Serial.print(" \t| stdx = ");
  Serial.print(stdx);
  Serial.print(" \t| drive = ");
  Serial.print(drive);
  */
  
  Serial.print(lraccx);
  Serial.print(",");
  Serial.print(lrgyrx);
  Serial.print(",");
  Serial.print(center);
  Serial.print(",");
  Serial.print(gcenter);
  Serial.print(",");
  Serial.print(angX);
  Serial.print(",");
  Serial.print(avgx);
  Serial.print(",");
  Serial.print(stdx);
  Serial.print(",");
  Serial.print(drive);
  
#endif

  //serialOutCSV();
  
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();

}

float getFloat( int val )
{
  return val;
}

void readChip()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  angX = ((float)(AcX) / ((float)sqrt( (long)((long)AcX*(long)AcX)+(long)((long)AcZ*(long)AcZ) ) ) ) * 256;
  //AcX = angX;
}


void DriveMotors( int torque )
{
  //int c_torque = constrain( abs(torque)*2, 80, 255 );
  int c_torque = constrain(map( abs(torque), 0, 10, 40, 255 ), 0, 255 );
  int wait = map(abs(torque), 0, 15, 10, 60 );
  
  Serial.print( "," );
  Serial.print( wait );
  Serial.print( "," );
  Serial.println( c_torque );
  
  if( torque > 0 )
  {
     analogWrite( 10, 0 );
     analogWrite( 11, c_torque );
     
     analogWrite( 5, 0 );
     analogWrite( 6, c_torque );
  }
  else if( torque < 0 )
  {
     analogWrite( 11, 0 );
     analogWrite( 10, c_torque );

     analogWrite( 6, 0 );
     analogWrite( 5, c_torque );
  }
  else
  {
     analogWrite( 11, 0 );
     analogWrite( 10, 0 );
    
     analogWrite( 6, 0 );
     analogWrite( 5, 0 );
  } 
  
  /*
  delay( wait );
  {
     analogWrite( 11, 0 );
     analogWrite( 10, 0 );
    
     analogWrite( 6, 0 );
     analogWrite( 5, 0 );
  } 
  */
}




void serialOut()
{
  Serial.print("AcX = "); Serial.print( getFloat(AcX));
  //Serial.print(" | AcY = "); Serial.print(getFloat(AcY));
  //Serial.print(" | AcZ = "); Serial.print(getFloat(AcZ));
  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(getFloat(GyX));
  //Serial.print(" | GyY = "); Serial.println(getFloat(GyY));
  //Serial.print(" | GyZ = "); Serial.println(getFloat(GyZ));
}

void serialOutCSV()
{
  Serial.print( millis() );
  Serial.print(","); Serial.print( getFloat(AcX));
  //Serial.print(","); Serial.print(getFloat(AcY));
  //Serial.print(" | AcZ = "); Serial.print(getFloat(AcZ));
  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(","); Serial.print(getFloat(GyX));
  //Serial.print(","); Serial.println(getFloat(GyY));
  //Serial.print(" | GyZ = "); Serial.println(getFloat(GyZ));
}



float Q_angle  =  .001; //0.001
float Q_gyro   =  .003;  //0.003
float R_angle  =  .003;  //0.03

float x_angle = 0;
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;	
float dt, y, S;
float K_0, K_1;

float kalmanCalculate(float newAngle, float newRate,int looptime) {
  dt = float(looptime)/1000.0;                                    // XXXXXXX arevoir
  x_angle += dt * (newRate - x_bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;
  
  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;
  
  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
  
  return x_angle;
}

  
// KasBot V1  -  PID module

#define   GUARD_GAIN   20.0
  
  //double aggK=0.5, aggKp=5, aggKi=.3, aggKd=2;
  
float last_error = 0;
float integrated_error = 0;
float pTerm = 0, iTerm = 0, dTerm = 0;

int updatePid(float targetPosition, float currentPosition, float K, float Kp, float Ki, float Kd)   
{
  float error = targetPosition - currentPosition; 
  pTerm = Kp * error;
  integrated_error += error;                                       
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd * (error - last_error);                            
  last_error = error;
  return -constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}  



