/*

  # Product: 6 DOF Sensor-MPU6050

  # SKU    : SEN0142

  # Description:

  # To read  accel/gyro data from 6 DOF Sensor

*/



#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "linear_regression.h"

MPU6050 accelgyro;
linear_regression lr_sonar(50);

int16_t ax, ay, az;  // define accel as ax,ay,az
int16_t gx, gy, gz;  // define gyro as gx,gy,gz

double fax, fay, faz;  // define accel as ax,ay,az
double fgx, fgy, fgz;  // define gyro as gx,gy,gz

//double ax, ay, az;  // define accel as ax,ay,az
//double gx, gy, gz;  // define gyro as gx,gy,gz

#define LED_PIN 13
#define ROLLER  07
#define TRIGGER 02
#define ECHO    04

#define I2C_SDA A4
#define I2C_SCL A5

bool blinkState = false;

// "Me" movement assignment
const int me_stop = 0;
const int me_forward = 1;
const int me_backward = 2;
const int me_left_forward = 3;
const int me_right_forward = 4;
const int me_left_backward = 5;
const int me_right_backward = 6;
const int me_spin_left = 7;
const int me_spin_right = 8;

int current_direction = me_stop;

const int drive = 255;

int roller_count = 0;
int roller_time = 0;
bool roller_toggle = false;

bool roller_check()
{
  if ( roller_toggle != digitalRead( ROLLER ) )
  {
    roller_time = millis();
    roller_toggle = !roller_toggle;
    roller_count++;
    return true;
  }
  else if ( millis() - roller_time < 1000 )
  {
    return true;
  }

  return false;
}

int sonar_ping( int trigger_pin, int echo_pin )
{
  int lms = 0;
  int hms = 0;
  int i;

  for ( i = 0; i < 3; i++ ) // try times to get a none negative value
  {
    int ts = micros();

    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(15);
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(15);
    digitalWrite(TRIGGER, LOW);

    while ( !digitalRead(ECHO) )
    {
      lms = micros();

      if ( lms > (ts + 8000) )
      {
        break;
      }
    }

    while ( digitalRead(ECHO) )
    {
      hms = micros();

      if ( hms > (lms + 8000) )
      {
        break;
      }
    }

    if ( (hms - lms) >= 0 )
      break;
  }

  int ret = (hms - lms);

  if ( ret < 0 )
    ret = 0;

  return ret;
}



int sonar_cm( int usecs )

{
  return (usecs / 59);
}



int sonar_in( int usecs )

{
  return (usecs / 145);
}


int sonar_dump_cm()
{
  int ping = sonar_ping( TRIGGER, ECHO );

  Serial.print( sonar_in(ping) );
  Serial.print("in : ");
  Serial.print( sonar_cm(ping) );
  Serial.println("cm");

  return sonar_cm(ping);
}

int movingDirection()
{
  return current_direction;
}
void moveMe( int way, int drive )
{

  current_direction = way;

  // out to motor drive PIN assignment
  const int left_motor_forward = 5;
  const int right_motor_forward = 9;
  const int left_motor_backward = 3;
  const int right_motor_backward = 6;

  analogWrite( left_motor_forward, 0 );
  analogWrite( right_motor_forward, 0 );
  analogWrite( left_motor_backward, 0 );
  analogWrite( right_motor_backward, 0 );

  switch ( way )
  {
    case me_stop:
      break;
    case me_forward:
      analogWrite( left_motor_forward, drive );
      analogWrite( right_motor_forward, drive );
      break;
    case me_backward:
      analogWrite( left_motor_backward, drive );
      analogWrite( right_motor_backward, drive );
      break;
    case me_left_forward:
      analogWrite( left_motor_forward, drive );
      break;
    case me_right_forward:
      analogWrite( right_motor_forward, drive );
      break;
    case me_left_backward:
      analogWrite( left_motor_backward, drive );
      break;
    case me_right_backward:
      analogWrite( right_motor_backward, drive );
      break;
    case me_spin_left:
      analogWrite( left_motor_backward, drive );
      analogWrite( right_motor_forward, drive );
      break;
    case me_spin_right:
      analogWrite( left_motor_forward, drive );
      analogWrite( right_motor_backward, drive );
      break;
  }
}
#include "pitches.h"

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void music() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
}


void setup() {
  music();
  
  Wire.begin();      // join I2C bus

  Serial.begin(115200);    //  initialize serial communication
  Serial.println("Initializing I2C devices...");

  accelgyro.initialize();

  // verify connection

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode(LED_PIN, OUTPUT);  // configure LED pin
  pinMode(TRIGGER, OUTPUT);  // configure sonar pin
  pinMode(ECHO,    INPUT);   // configure sonar pin
  pinMode(ROLLER,  INPUT);   // configure sonar pin

  pinMode( 3, OUTPUT );
  pinMode( 5, OUTPUT );
  pinMode( 6, OUTPUT );
  pinMode( 9, OUTPUT );


}


double getDistsance()
{
  for ( int i = 0; i <= 50; i++ )
  {
    lr_sonar.log_entry( micros(), sonar_cm( sonar_ping(TRIGGER, ECHO) ) );
  }
  return lr_sonar.avg();
}

void scan_area( int time_unit )
{
  moveMe( me_stop, drive );
  moveMe( me_backward, drive );
  delay(time_unit * 50);

  moveMe( me_backward, drive );
  delay(time_unit * 30);

  if ( rand() % 2 == 0 )
  {
    for ( int i = 0; i < 4; i++ )
    {
      moveMe( me_spin_right, drive );
      delay(time_unit * 50);
      moveMe( me_stop, drive );
      delay(time_unit * 50 );
      double distance = getDistsance();
      if ( distance > 30 )
        return;
    }
  }
  else
  {
    for ( int i = 0; i < 4; i++ )
    {
      moveMe( me_spin_left, drive );
      delay(time_unit * 50);
      moveMe( me_stop, drive );
      delay(time_unit * 50 );
      double distance = getDistsance();
      if ( distance > 30 )
        return;
    }
  }
}


void loop() {

  int drive = 255;

  //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  lr_sonar.log_entry( micros(), sonar_cm( sonar_ping(TRIGGER, ECHO) ) );

  double acm = lr_sonar.avg();
  double lcm = lr_sonar.calc();

  fax = ax;
  fay = ay;
  faz = az;
  fgx = gx;
  fgy = gy;
  fgz = gz;

  Serial.print( acm );
  Serial.print( "\t" );
  Serial.println( lcm );


  if ( movingDirection() == me_forward )
  {
    if ( !roller_check() )
    {
      //acm = 0; // fool sonar
    }
  }

  if ( acm < 25 )
  {
    moveMe( me_backward, 255 );
    delay(10);
    moveMe( me_stop, 255 );
    music();
    scan_area(15);
  }
  else
  {
    if ( millis() % 3000 == 0 )
    {
      if ( rand() % 2 == 0 )
      {
        moveMe( me_spin_left, 255 );
      }
      else
      {
        moveMe( me_spin_right, 255 );
      }
      delay(200);
      scan_area(5);
    }
    moveMe( me_forward, drive );
  }
}


