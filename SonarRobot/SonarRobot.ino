// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
int tmark = 0;
int bmark = 0;
void setup() {
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  
}

void loop() {
  delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int pingSonar = sonar.ping_cm();
  Serial.print("Ping: ");
  Serial.print(pingSonar); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  
  
  int ms = millis();
  
  if( pingSonar <= 1 || pingSonar >= 20 )
  {
    analogWrite(5, 200);    
    analogWrite(3, 200);    
    delay(1000);
  }
  else if( pingSonar < 20 && pingSonar > 10 ) //turn
  {
    //if( tmark < millis() )
    //  tmark = millis() + 700;

    analogWrite(5, 200);    
    analogWrite(3, 50);

    delay(1000);    
  }
  else if( pingSonar <= 10 )   // back up
  {
//    if( bmark < millis() )
//      bmark = millis() + 1500;

    analogWrite(5, 50);    
    analogWrite(3, 50);  
  
    delay( 1000 );  
  }
  
  
}
