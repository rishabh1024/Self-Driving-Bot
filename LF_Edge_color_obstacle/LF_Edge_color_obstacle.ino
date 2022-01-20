int left1= 2, left2= 4, right1= 7, right2= 8, enable1=3, enable2= 5,ir3=12,ir4=11,ir1=10,ir2=13;

unsigned long start1,finish1,delta1,start2,finish2,delta2,start3,finish3,delta3,start4,finish4,delta4;

//color sensor conections
const int s0 = 9;  
const int s1 = A1;  
const int s2 = A2;  
const int s3 = A3;  
const int out = A4;   
// LED pins connected to Arduino
int redLed = 2;  
int yellowLed = 3;  
int blueLed = 4;
// Variables  
int red = 0;  
int green = 0;  
int blue = 0;  

//ultrasonic connections
const int trigPin = A5;
const int echoPin = A0;

long duration;
int distance;
int flag;
  
void setup() {
  // put your setup code here, to run once:
  pinMode(ir3,INPUT);
  pinMode(ir4,INPUT); 
  pinMode(ir2,INPUT);
  pinMode(ir1,INPUT);
  pinMode(left1,OUTPUT);
  pinMode(left2,OUTPUT);
  pinMode(right1,OUTPUT);
  pinMode(right2,OUTPUT);
  pinMode(enable1,OUTPUT);  
  pinMode(enable2,OUTPUT);
  digitalWrite(enable1,HIGH);
  digitalWrite(enable2,HIGH);
  pinMode(trigPin,OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin,INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); 
  pinMode(s0, OUTPUT);  
  pinMode(s1, OUTPUT);  
  pinMode(s2, OUTPUT);  
  pinMode(s3, OUTPUT);  
  pinMode(out, INPUT);  
  pinMode(redLed, OUTPUT);  
  pinMode(yellowLed, OUTPUT);  
  pinMode(blueLed, OUTPUT);  
  digitalWrite(s0, HIGH);  
  digitalWrite(s1, HIGH);  
}

void dist()
{
   // Clears the trigPin
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance= duration*0.034/2;

  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
}

void Stop()
{
  digitalWrite(left2,LOW);
  digitalWrite(right2,LOW);
  digitalWrite(left1,LOW); // Stop
  digitalWrite(right1,LOW); 
}

void forward()
{
   analogWrite(enable1,110);
   analogWrite(enable2,125);
   digitalWrite(left2,HIGH);
   digitalWrite(right2,HIGH); //Forward
   digitalWrite(left1,LOW);
   digitalWrite(right1,LOW);
}

void forward1()
{
   analogWrite(enable1,52.4);
   analogWrite(enable2,55);
   digitalWrite(left2,HIGH);
   digitalWrite(right2,HIGH); //Forward
   digitalWrite(left1,LOW);
   digitalWrite(right1,LOW);
}
void left()
{ 
  digitalWrite(left2,LOW);
  digitalWrite(right2,HIGH); // Left
  digitalWrite(left1,LOW);
  digitalWrite(right1,LOW); 
}

void right()
{
  digitalWrite(left2,HIGH);
  digitalWrite(right2,LOW);
  digitalWrite(left1,LOW); //Right
  digitalWrite(right1,LOW);
}

void rright()
{
  dist();
  flag=1;
  Stop();
  start1 = millis();
  while(distance<23)
  {
    rev1();
    dist();
  }
  finish1 = millis();
  delta1 = finish1 - start1;
  Stop();
  delay(1000);
  if(flag==1)
  {
    rev2();
    delay(1.2*delta1);
    flag=0;
  }
  Stop();
  delay(1000);
}

void rleft()
{
  dist();
  int flag1=1;
  Stop();
  start2 = millis();
  while(distance<23)
  {
    rev2();
    dist();
  }
  finish2 = millis();
  delta2 = finish2 - start2;
  Stop();
  delay(1000);
  if(flag1==1)
  {
    rev1();
    delay(delta2);
    flag1=0;
  }
  Stop();
  delay(1000);   

}

void LF()
{
  if (digitalRead(ir3)==0 && digitalRead(ir4)==0)
  {
    forward();
  }
  if (digitalRead(ir3)==0 && digitalRead(ir4)==1)
  {
    right();
  }
  if (digitalRead(ir3)==1 && digitalRead(ir4)==0)
  {
    left();    
  }
  if (digitalRead(ir3)==1 && digitalRead(ir4)==1)
  {
    Stop();
  }
}

void rev1()
{
  digitalWrite(left2,HIGH);
  digitalWrite(right2,LOW); //Onthespotright
  digitalWrite(left1,LOW);
  digitalWrite(right1,HIGH);
}

void rev2()
{
  digitalWrite(left2,LOW);
  digitalWrite(right2,HIGH); //Onthespotleft
  digitalWrite(left1,HIGH);
  digitalWrite(right1,LOW);
}

void curveleft()
{
  analogWrite(enable1,91);
  analogWrite(enable2,90);
  digitalWrite(left2,HIGH);
  digitalWrite(right2,HIGH); //Forward
  digitalWrite(left1,LOW);
  digitalWrite(right1,LOW);
  delay(2000);        
}

void curveright()
{
  analogWrite(enable1,90);
  analogWrite(enable2,104);
  digitalWrite(left2,HIGH);
  digitalWrite(right2,HIGH); //Forward
  digitalWrite(left1,LOW);
  digitalWrite(right1,LOW);
  delay(2000);        
}

void color()  
{    
  digitalWrite(s2, LOW);  
  digitalWrite(s3, LOW);  
  //count OUT, pRed, RED  
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s3, HIGH);  
  //count OUT, pBLUE, BLUE  
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s2, HIGH);  
  //count OUT, pGreen, GREEN  
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
}

void edge()
{
  color();
  if ((digitalRead(ir2)==1 && digitalRead(ir1)==1)||(red > 50 && red < 95   &&  green > 35 && green < 70    &&  blue > 45 && blue < 85))
  {
    forward();
  }
  if (digitalRead(ir2)==0 && digitalRead(ir1)==1)
  {
    left(); 
  }
  if (digitalRead(ir2)==1 && digitalRead(ir1)==0)
  {
        right();
  }
  if ((digitalRead(ir2)==0 && digitalRead(ir1)==0)||(red > 12 && red < 30   &&  green > 40 && green < 70    &&  blue > 33 && blue < 70))
  {
    Serial.println("Randi IR");
    Stop();
    delay(1000);
  }
  if(red > 10 && red < 20   &&  green > 10 && green < 25    &&  blue > 20 && blue < 38)
  {
   forward1();
  }
}

void avoid()
{
    dist();
    Stop();
    delay(500);
    rright();
    distance=0;
    rleft();    
    if(delta1<delta2)
    {
       rev1();
       delay(delta1);
       forward();
       delay(5*delta1);
       rev2();
       delay(2*delta1);
       forward();
       delay(3.5*delta1);
       curveleft();
       forward();
       //delta1=0;
       
    }
    else 
      rev2();
      delay(delta2);
      forward();
      delay(5.5*delta2);
      rev1();
      delay(1.2*delta2);
      forward();
      delay(1.8*delta2);
      curveright();

}
void loop() 
{  
       start:
       dist();
       edge();
       LF();
       if(distance<18)
       {
        avoid();
       }  
}



