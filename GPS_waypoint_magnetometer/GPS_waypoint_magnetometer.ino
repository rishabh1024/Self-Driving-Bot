  /*********************
 *10 to GPS Module TX*
 *09 to GPS Module RX*
 *********************/
int left1= 2, left2= 4, right1= 7, right2= 8, enable1=3, enable2= 5, ir3=12, ir4=11, ir1=10, ir2=13;

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include<Wire.h>
#include<HMC5883L.h>

float currentHeading;
float targetHeading; 
float headingError; 
#define HEADING_TOLERANCE 5  

SoftwareSerial mySerial(A14,A15);
TinyGPS gps; 

HMC5883L compass;

int error=0;

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
#define NUMBER_WAYPOINTS 2          // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
float waypointLat[NUMBER_WAYPOINTS] = {28.52402,28.52416};
float waypointLong[NUMBER_WAYPOINTS]= {77.57155,77.57160};

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

void setup()  
{
  
  // Oploen serial communications and wait for port to open:
  Serial.begin(9600);
  Wire.begin(); // Start the I2C interface.

  compass = HMC5883L(); // Construct a new HMC5883 compass.
  error = compass.SetScale(1.3); // Set the scale of the compass to 1.3Ga

  if(error != 0)
  { 
    // If there is an error, print it out. 
    Serial.println(compass.GetErrorText(error));
    error =0;
  }

  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous

  if(error != 0) 
  {
    // If there is an error, print it out.
      Serial.println(compass.GetErrorText(error)); //Todo: Error handling for this method in .h and .cpp
    error=0;
  }
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
   analogWrite(enable1,250);
   analogWrite(enable2,250); 
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
   analogWrite(enable1,150);
   analogWrite(enable2,150); 
   digitalWrite(left2,HIGH);
   digitalWrite(right2,HIGH); //Forward
   digitalWrite(left1,LOW);
   digitalWrite(right1,LOW);
}

void left()
{ 
  analogWrite(enable1,170);
   analogWrite(enable2,170); 
  digitalWrite(left2,LOW);
  digitalWrite(right2,HIGH); // Left
  digitalWrite(left1,LOW);
  digitalWrite(right1,LOW); 
}

void right()
{
  analogWrite(enable1,170);
  analogWrite(enable2,170); 
  digitalWrite(left2,HIGH);
  digitalWrite(right2,LOW);
  digitalWrite(left1,LOW); //Right
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

int readcompass()
{
  // Retrieve the raw values from the magnetometer (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrieve the scaled values from the magnetometer (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();

  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  // Atan2() automatically check the correct formula taking care of the quadrant you are in
  float heading = atan2(scaled.YAxis, scaled.XAxis);

  // Once you have your heading, you must then add your 'Declination Angle',
  // which is the 'Error' of the magnetic field in your location. Mine is 0.0404 
  // Find yours here: http://www.magnetic-declination.com/
  
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0206530628;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Output the data via the serial port.
  //Output(raw, scaled, heading, headingDegrees);

  // By default the HMC5883L reads the data 15 time per second (15Hz)
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
  // delay(66);
  return (headingDegrees);
}

int courseToWaypoint() 
{
  float dlon = radians(targetLong-currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
  return targetHeading;
}   // courseToWaypoint()

void calcDesiredTurn(void)
{
    // calculate where we need to turn to head to destination
    headingError = targetHeading - currentHeading;
    
    // adjust for compass wrap
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
  
    // calculate which way to turn to intercept the targetHeading
    if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
    {  
      forward();  
    }
    else if (headingError < 0)
    {
      left();
    }
    else if (headingError > 0)
      right;
    else
      forward();
 
}
  
void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
{
  Serial.print("Raw:\t");
  Serial.print(raw.XAxis);
  Serial.print("   ");   
  Serial.print(raw.YAxis);
  Serial.print("   ");   
  Serial.print(raw.ZAxis);
  Serial.print("   \tScaled:\t");

  Serial.print(scaled.XAxis);
  Serial.print("   ");   
  Serial.print(scaled.YAxis);
  Serial.print("   ");   
  Serial.print(scaled.ZAxis);

  Serial.print("   \tHeading:\t");
  Serial.print(heading);
  Serial.print(" Radians   \t");
  Serial.print(headingDegrees);
  Serial.println(" Degrees   \t");
  delay(5000);
}

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
    Serial.print("Distance to waypoint:"); Serial.print(dis); Serial.println(" meters");
    currentHeading=readcompass();
    targetHeading=courseToWaypoint();
    Serial.print("Current Heading :"); Serial.println(currentHeading);
    Serial.print("Target Heading : "); Serial.print(targetHeading);
    if(dis<4.00)
    {
      slow();
      calcDesiredTurn();
    }
    if(dis >= WAYPOINT_DIST_TOLERANE)
    {
     forward();
     calcDesiredTurn();
    }
    else if(dis<=WAYPOINT_DIST_TOLERANE)
    {    
     Stop();
     delay(2000);
     i++;
     forward();
     calcDesiredTurn();
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
