/*
 Fade
 
 This example shows how to fade an LED on pin 9
 using the analogWrite() function.
 
 This example code is in the public domain.
 */

int led = 11;           // the pin that the LED is attached to
int brightness = 100;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

// the setup routine runs once when you press reset:
void setup()  { 
  // declare pin 9 to be an output:
  pinMode(11, OUTPUT);
  pinMode(11, OUTPUT);
  Serial.begin(9600);
} 

// the loop routine runs over and over again forever:
void loop()  { 
  
  int d = 1000;
  analogWrite( 10, 0 );
  analogWrite(11, 255);
  delay(2000);
  analogWrite(11, 0);
  analogWrite(10, 255);
  delay(2000);
  analogWrite(10, 0);
  delay(2000);
  
  for( int duty=100; duty <= 255; duty += 10 )
  {
    // set the brightness of pin 9:
    analogWrite(11, duty);
    Serial.println(duty);
    delay(d);    
  }

  for( int duty=255; duty >= 100; duty -= 10 )
  {
    // set the brightness of pin 9:
    analogWrite(11, duty);
    Serial.println(duty);
    delay(d);    
  }

  analogWrite( 11, 0 );

  for( int duty=100; duty <= 255; duty += 10 )
  {
    // set the brightness of pin 9:
    analogWrite(10, duty);
    Serial.println(duty);
    delay(d);    
  }

  for( int duty=255; duty >= 100; duty -= 10 )
  {
    // set the brightness of pin 9:
    analogWrite(10, duty);
    Serial.println(duty);
    delay(d);    
  }

}

