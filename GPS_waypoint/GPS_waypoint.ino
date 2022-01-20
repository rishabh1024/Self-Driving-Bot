  /*********************
 *10 to GPS Module TX*
 *09 to GPS Module RX*
 *********************/
int left1= 2, left2= 4, right1= 7, right2= 8, enable1=3, enable2= 5, ir3=12, ir4=11, ir1=10, ir2=13;

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include<Wire.h>


SoftwareSerial mySerial(A8,A9); //TX,RX
TinyGPS gps; 



float currentLat,
      currentLong,
      targetLat,
      targetLong;
float distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it/


long lat, lon;
float flat, flon;
unsigned long age, date, time, chars;
int year;
byte month, day, hour, minute, second, hundredths;
unsigned short sentences, failed;
  
#define WAYPOINT_DIST_TOLERANE  2   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 3          // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
float waypointLat[NUMBER_WAYPOINTS] = {28.52407,28.52410,28.52416};
float waypointLong[NUMBER_WAYPOINTS]= {77.57158,77.57159,77.57160};

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

void setup()  
{
  
  // Oploen serial communications and wait for port to open:
  Serial.begin(9600);
  Wire.begin(); // Start the I2C interface.
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  delay(1000);
  Serial.println("uBlox Neo 6M");
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.print("Sizeof(gpsobject) = "); 
  Serial.println(sizeof(TinyGPS));
  pinMode(left1,OUTPUT);
  pinMode(left2,OUTPUT);
  pinMode(right1,OUTPUT);
  pinMode(right2,OUTPUT);
  pinMode(enable1,OUTPUT);  
  pinMode(enable2,OUTPUT);
  digitalWrite(enable1,HIGH);
  digitalWrite(enable2,HIGH);
  Serial.println(); 
}

void forward()
{ 
   analogWrite(enable1,200);
   analogWrite(enable2,172); 
   digitalWrite(left2,HIGH);
   digitalWrite(right2,HIGH); //Forward
   digitalWrite(left1,LOW);
   digitalWrite(right1,LOW);
}

void Stop()
{
   digitalWrite(left2,LOW);
   digitalWrite(right2,LOW); //Stop
   digitalWrite(left1,LOW); 
   digitalWrite(right1,LOW);
}

void slow()
{
   analogWrite(enable1,148.5);
   analogWrite(enable2,96); 
   digitalWrite(left2,HIGH);
   digitalWrite(right2,HIGH); //Forward
   digitalWrite(left1,LOW);
   digitalWrite(right1,LOW);
}

void rright()
{
  digitalWrite(left2,HIGH);
  digitalWrite(right2,LOW); //Onthespotright
  digitalWrite(left1,LOW);
  digitalWrite(right1,HIGH);
  
}

void rleft()
{
  digitalWrite(left2,LOW);
  digitalWrite(right2,HIGH); //Onthespotleft
  digitalWrite(left1,HIGH);
  digitalWrite(right1,LOW);
  
}

float distanceToWaypoint(int i) 
{
    int R=6371000; // Radius of Earth
    //Setting co-ordinates of current waypoint as targets
    targetLat = waypointLat[i];  
    targetLong = waypointLong[i]; 
    
    float deltalong = currentLong - targetLong;
    float deltalat = currentLat-targetLat;
    float a= (sq(sin(deltalat/2))+cos(targetLat)*cos(currentLat)*(sq(sin(deltalong/2))));
    float c=2*atan2(sqrt(a), sqrt(1-a));
    distanceToTarget = c*R*10; 
     
    // check to see if we have reached the current waypoint
    //if (distanceToTarget <= WAYPOINT_DIST_TOLERANE)
      //nextWaypoint();
    
  return distanceToTarget;
}  // distanceToWaypoint()



int i=0;

void loop() // run over and over
{
  bool newdata = false;
  unsigned long start = micros();
  // Every 5 seconds we print an update
  while (micros() - start < 5000) 
  {
    if (mySerial.available()) 
    {
      char c = mySerial.read();
      Serial.print(c);  // uncomment to see raw GPS data
      if (gps.encode(c)) 
      {
        newdata = true;
        break;  // uncomment to print new data immediately!
      }
    }
  }
  
  if (newdata) 
  {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    currentLat=flat;
    currentLong=flon;
    float dis=distanceToWaypoint(i);
    Serial.print("Distance to waypoint:"); Serial.print(dis); Serial.print(" meters");
    char ch=Serial.read();
    while(Serial.available())
    {
      if (ch=='l')
      {
        rleft();
        delay(100);
      }
    }
    if(dis >= 4.00)
    {
     forward();
    }
    else if( (dis<=4.00) && (dis>WAYPOINT_DIST_TOLERANE))
    {    
      slow();   
    } 
    else if(dis<WAYPOINT_DIST_TOLERANE)
    {
     Stop();
     delay(1000);
     //rright();
     i++;
     forward();
    }
    
    Serial.println("-------------");
    Serial.println();
  }
  
  if(i==NUMBER_WAYPOINTS)
  {
    Stop();
    exit(0);
  }
  
}

void gpsdump(TinyGPS &gps)
{


  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/  Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  // On Arduino, GPS characters may be lost during lengthy Serial.print()
  // On Teensy, Serial prints to USB, which has large output buffering and
  //   runs very fast, so it's not necessary to worry about missing 4800
  //   baud GPS characters.

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
    Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): ");
    Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
  Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour+5.5));  Serial.print(":"); //Serial.print("UTC +08:00 Malaysia");
  Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
  Serial.print("."); Serial.print(static_cast<int>(hundredths)); Serial.print(" UTC + 05:30 India");
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");

  Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");
  Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  Serial.print("Alt(float): "); printFloat(gps.f_altitude()); Serial.print(" Course(float): ");
  printFloat(gps.f_course()); Serial.println();
  Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.print(" (mph): ");
  printFloat(gps.f_speed_mph());
  Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): ");
  printFloat(gps.f_speed_kmph()); Serial.println();

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
  Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) 
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) 
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}
