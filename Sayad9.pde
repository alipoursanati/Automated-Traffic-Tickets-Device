/*
Auto-Ticketing System(BASIR Project)
By: Ali Rajabour Sanati; Ali Boroumand Telgerd
Buzzer->12; GreenLED->4; RedLED->11 GPS->(TX:16-RX:17) GSM->18 
*/
//#include <NewSoftSerial.h>
#include <TinyGPS.h>
#include "pitches.h"
int SIM900=18;
#define NUM_OFF 2
#define DELAY 50
#define TEST_BUTTON 9
#define DEFAULT_SPEED_LIMIT 0
#define NSPEEDZONES 3
int year;
byte month, day, hour, minute, second;
unsigned long chars;
boolean debugMode = true;
boolean speeding = false;
//int red[4];
class Vertex {
public:
  Vertex(float llat, float llng) {
    lat = llat;
    lng = llng;
  }
  float lat;
  float lng;
};

class SpeedZone {
public:
  SpeedZone(int s) {
    speedLimit = s;
  }
  void setVertices(int n, Vertex *v) {
    nVertices = n;
    vertices = v;
  }
  int nVertices;
  Vertex *vertices;
  int speedLimit;
};
SpeedZone *speedZones[NSPEEDZONES];
TinyGPS gps;
//NewSoftSerial nss(2, 3);
boolean buttonPressed = false;

//BUZZER
int buzzer=12;
int tones[] = {340, 340, 340, 340, 340, 340};
int tonesDurations[] = {2,2,2,2,2,2};

//LED
int led=4;
int rled=11;

//RESET
int PINtoRESET=9;

void setup() {
  pinMode(PINtoRESET, INPUT);
  digitalWrite(PINtoRESET, LOW);
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  setupSpeedZones();
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(rled, OUTPUT);
  setupSpeedZones();
  pinMode(TEST_BUTTON, INPUT);
  digitalWrite(TEST_BUTTON, HIGH);

  randomSeed(analogRead(0));

  // Allow GPS to power up
  delay(2000);

  // Establish serial connection to GPS
  //nss.begin(9600);
}



void setupSpeedZones() {
  // GHAFARI
  speedZones[0] = &SpeedZone(15);
  speedZones[0]->setVertices(4, (Vertex[4]){
      Vertex(32.87,59.23),
      Vertex(32.84,59.23),
      Vertex(32.84,59.22),
      Vertex(32.87,59.23)});

  // IRAN
  speedZones[1] = &SpeedZone(50);
  speedZones[1]->setVertices(4, (Vertex[4]){
      Vertex(39.67,43.80),
      Vertex(37.74,60.98),
      Vertex(27.46,61.54),
      Vertex(28.18,46.39)});
  
		
  // Bounding rectangle for residential areas
  // This should be defined last in the list because it's the "catch all" speed zone.
  speedZones[2] = &SpeedZone(45);
  speedZones[2]->setVertices(4, (Vertex[4]){
      Vertex(45.045783186920296, -93.48395347595215),
      Vertex(45.0456315793546, -93.44983577728271),
      Vertex(45.02585851043667, -93.45009326934814),
      Vertex(45.02594950646131, -93.48326683044434)});


  if (debugMode) {
    printSpeedZones();
  }

}

/*
 * This is the point-in-polygon algorithm adapted from
 * http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
 */
boolean inSpeedZone(int speedZone, float lat, float lng) {
  SpeedZone *s = speedZones[speedZone];

  int i, j;
  boolean inside = false;
  for (i = 0, j = s->nVertices-1; i < s->nVertices; j = i++) {
    if ( ((s->vertices[i].lat > lat) != (s->vertices[j].lat > lat)) &&
         (lng < (s->vertices[j].lng - s->vertices[i].lng) * (lat - s->vertices[i].lat) / (s->vertices[j].lat - s->vertices[i].lat) + s->vertices[i].lng) )
       inside = !inside;
  }

  return inside;
}


boolean inSpeedZone(int speedZone) {
  float lat, lng;
  unsigned long  fix_age;

  // retrieves +/- lat/long in 100,000ths of a degree
  gps.f_get_position(&lat, &lng, &fix_age);

  return inSpeedZone(speedZone, lat, lng);
}

void loop() {

  /*if (readGPS()) {
    if (debugMode) {
      debug();
    }
    speeding = isSpeeding();
  }

 if (digitalRead(TEST_BUTTON) == LOW) {
    buttonPressed = true;
  } else {
    buttonPressed = false;
  }*/
  readGPS();
  //debug();
while(true){
   /*if (readGPS()) {
    if (debugMode) {
      debug();
    }
    speeding = isSpeeding();
  }*/
  debug();
  if (isSpeeding()) {
  buzzing();
  delay(3000);
  } else {
 redlight();
  }
  if (isSpeeding()) {  
    sendTicket();
    delay(1000);
    pinMode(PINtoRESET, OUTPUT);
    while(1);
  } else {
 greenlight();
  }
//return true;
}
}
bool readGPS() {
  while (Serial2.available()) {
    if (gps.encode(Serial2.read())) {
      return true;
    }
  }
  return false;
}
/*void readGPS() {
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.print(c);
    gps.encode(c);
    Serial.println("test");
    delay(5000);
    //c=0;
   
    }
  }}*/

  
int getSpeedLimit() {
  boolean isInSpeedZone;

  for(int i=0;i<NSPEEDZONES;i++) {
    isInSpeedZone = inSpeedZone(i);
    if (isInSpeedZone) {
      return speedZones[i]->speedLimit;
    }
  }
  return DEFAULT_SPEED_LIMIT;
}

boolean isSpeeding() {
  int speed = (int)(gps.f_speed_mph() + 0);
  int speedLimit = getSpeedLimit();

  if (speed > speedLimit) {
    return true;
  }
  return false;
}
void buzzing() {
  for(int ib=0l; ib<1; ib++){
  for (int Note = 0; Note < 6; Note++) {
  int noteDuration = 1000/tonesDurations[Note];
  tone(buzzer, tones[Note],noteDuration);
  int pauseBetweenNotes = noteDuration * 1.30;
  delay(pauseBetweenNotes);
  noTone(buzzer);
}}} 

void redlight(){ 
 for (int ir=0; ir<4; ir++){ 
 digitalWrite(rled, HIGH);
 delay(500);
 digitalWrite(rled,LOW);
 delay(500);
}}
  
void greenlight(){  
 for(int ig=0; ig<3; ig++){
 digitalWrite(led, HIGH);
 delay(500);
 digitalWrite(led,LOW);
 delay(500);
}}
  
void sendTicket(){
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age); 
for(int it=0; it<1; it++){  
    Serial1.print("AT+CMGF=1\r"); 
    delay(400);
    Serial1.println("AT+CMGS=09155209831");// recipient's mobile number with country code
    delay(300);
    Serial1.print("Latitude = ");
    Serial1.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial1.print(" Longitude = ");
    Serial1.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial1.println("Pelak=44H345-i36");
    gps.crack_datetime(&year,&month,&day,&hour,&minute,&second);
    Serial1.print("Date: ");
    Serial1.print(month, DEC);
    Serial1.print("/");
    Serial1.print(day, DEC);
    Serial1.print("/");
    Serial1.println(year);
    Serial1.print("Time: ");
    Serial1.print(hour+3, DEC);
    Serial1.print(":");
    Serial1.print(minute+30, DEC);
    Serial1.print(":");
    Serial1.println(second, DEC);
    Serial1.print("Speed: ");
    Serial1.println(gps.f_speed_kmph());
    Serial1.print("Direction: ");
    Serial1.print(gps.cardinal(gps.f_course()));
    Serial1.println("Created By:");
    Serial1.println("Ali Rajabpour");
    Serial1.println("Ali Boroumand");
    delay(5000);
    Serial1.println((char)26); // End AT command with a ^Z, ASCII code 26
    delay(5000);
} }
/*
boolean policeLights() {

  // The white LEDs are so bright we actually turn them down quite a bit.
  // 0 is full brightness and 255 is off (because the cathode is connected to the PWM pin)
  //analogWrite(headlight2, 200);
  //analogWrite(headlight1, 200); 

  allOn();

  // Turn off some of the red and blue lights to give them a flashing effect.
  for(int i=0;i<NUM_OFF;i++) {
    digitalWrite(red[random(4)], HIGH);
  }
  delay(DELAY);

}

void allOn() {
  for(int i=0;i<4;i++) {
    digitalWrite(red[i], LOW);
  }
}

void allOff() {
  for(int i=0;i<4;i++) {
    digitalWrite(red[i], HIGH);
  }

}*/

void printSpeedZones() {

  for(int i=0;i<NSPEEDZONES;i++) {
    SpeedZone *s = speedZones[i];
    Serial.println(s->speedLimit);
    for(int v=0;v<s->nVertices;v++) {
      Serial.print("(");
      Serial.print(s->vertices[v].lat);
      Serial.print(", ");
      Serial.print(s->vertices[v].lng);
      Serial.println(")");
    }
  }
}

void debug() {
  long lat, lon;
  unsigned long fix_age, time, date, speed, course;

  // retrieves +/- lat/long in 100000ths of a degree
  gps.get_position(&lat, &lon, &fix_age);

  Serial.println(getSpeedLimit());

  Serial.print("lat: ");
  Serial.print(lat);
  Serial.print("    lng: ");
  Serial.print(lon);
  Serial.print("    speed: ");
  Serial.println(gps.f_speed_mph());
}

