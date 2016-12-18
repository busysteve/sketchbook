// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
int led = 13;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int STD_LOOP_TIME = 9; 
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

float center = 1500.00;

int avg_array[5];
int avg_mod = 0;

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
  Serial.begin(9600);
  
  delay(2000);
  
  // Blink 3 time for three seconds
  for( int i = 0; i < 3; i++ )
  {
      digitalWrite( led, HIGH );
      delay(100);
      digitalWrite( led, LOW );
      delay(900);
  }
  
  // Blink while sampling
  for( int i = 0; i < 5; i++ )
  {
      digitalWrite( led, HIGH );
      readChip();
      delay(10);
      digitalWrite( led, LOW );
      delay(90);
      avg_array[i] = AcY;
  }
  
  center = 0;
  for( int i = 0; i < 5; i++ )
  {
      center += avg_array[i];
  }
  center /= 5;
  
  // Blink 3 time for three seconds
  for( int i = 0; i < 3; i++ )
  {
      digitalWrite( led, HIGH );
      delay(100);
      digitalWrite( led, LOW );
      delay(900);
  }
  
}
void loop(){
  readChip();
  
  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(30);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    
  avg_array[avg_mod%10] = AcY;
  int accy = 0;
  
  for( int i = 0; i < 5; i++ )
  {
      accy += avg_array[i];
  }
  accy /= 5;
  
  //accy = kalmanCalculate(AcY, GyY, lastLoopTime);
  
#if 1
  if( accy > ( center * (center/accy) ) )
  {
     analogWrite( 10, 0 );
     analogWrite( 11, 255 );
     
     analogWrite( 5, 0 );
     analogWrite( 6, 255 );
  }
  else if( accy < ( center * (center/accy) ) )
  {
     analogWrite( 11, 0 );
     analogWrite( 10, 255 );

     analogWrite( 6, 0 );
     analogWrite( 5, 255 );
  }
  else if( 0 && accy > center + 100.00 )
  {
     analogWrite( 10, 0 );
     analogWrite( 11, 190 );
     
     analogWrite( 5, 0 );
     analogWrite( 6, 190 );
  }
  else if( 0 && accy < center - 100.00 )
  {
     analogWrite( 11, 0 );
     analogWrite( 10, 190 );

     analogWrite( 6, 0 );
     analogWrite( 5, 190 );
  }
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
  
}

void serialOut()
{
  //Serial.print("AcX = "); Serial.print( getFloat(AcX));
  Serial.print(" | AcY = "); Serial.print(getFloat(AcY));
  //Serial.print(" | AcZ = "); Serial.print(getFloat(AcZ));
  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  //Serial.print(" | GyX = "); Serial.print(getFloat(GyX));
  Serial.print(" | GyY = "); Serial.println(getFloat(GyY));
  //Serial.print(" | GyZ = "); Serial.println(getFloat(GyZ));
}

void serialOutCSV()
{
  Serial.print( millis() );
  //Serial.print("AcX = "); Serial.print( getFloat(AcX));
  Serial.print(","); Serial.print(getFloat(AcY));
  //Serial.print(" | AcZ = "); Serial.print(getFloat(AcZ));
  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  //Serial.print(" | GyX = "); Serial.print(getFloat(GyX));
  Serial.print(","); Serial.println(getFloat(GyY));
  //Serial.print(" | GyZ = "); Serial.println(getFloat(GyZ));
}



   float Q_angle  =  0.001; //0.001
    float Q_gyro   =  0.003;  //0.003
    float R_angle  =  0.03;  //0.03

    float x_angle = 0;
    float x_bias = 0;
    float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;	
    float dt, y, S;
    float K_0, K_1;

  float kalmanCalculate(float newAngle, float newRate,int looptime) {
    dt = float(looptime)/1000;                                    // XXXXXXX arevoir
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
