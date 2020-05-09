

//Valerio.raminelli@tin.it
//version with front obstacle detection
//including check direction with sonar on servo

#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo

#include <NewPing.h>

#define TRIGGER_PIN A5 //13 //A5//12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN    A4 //11 //A4// 11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 60 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_FrontPIN  2     //A5//13 //A5//12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_FrontPIN     4    //A4//11 //A4// 11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE_Front 60 // Maximum dist
#define laser 13
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar_Front(TRIGGER_FrontPIN, ECHO_FrontPIN, MAX_DISTANCE_Front); // NewPing setup of pins and maximum distance.

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

#define carSpeed 200 //200 Normal speed, max speed 250
#define X 100

float rightDistance = 0, leftDistance = 0, middleDistance = 0, frontDistance=0;
long int sonar_midle_ang [190];
float newDirection=90;
bool obstacle;



void forward(int acc){ 
  analogWrite(ENA, carSpeed+acc);
  analogWrite(ENB, carSpeed+acc);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Forward");
}

void back(int acc) {
  analogWrite(ENA, carSpeed+acc);
  analogWrite(ENB, carSpeed+acc);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Back");
}

void left(int acc) {
  analogWrite(ENA, carSpeed+acc);
  analogWrite(ENB, carSpeed+acc);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("Left");
}

void left10(int acc) {
  analogWrite(ENA, carSpeed+acc);
  analogWrite(ENB, carSpeed+acc);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("Left10");
}


void left45(int acc) {
  analogWrite(ENA, carSpeed+acc);
  analogWrite(ENB, carSpeed+acc);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("Left45");
}

void right(int acc) {
  analogWrite(ENA, carSpeed+acc);
  analogWrite(ENB, carSpeed+acc);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right");
}

void right10(int acc) {
  analogWrite(ENA, carSpeed+acc);
  analogWrite(ENB, carSpeed+acc);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right10");
}

void right45(int acc) {
  analogWrite(ENA, carSpeed+acc);
  analogWrite(ENB, carSpeed+acc);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right45");
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
} 

//Ultrasonic distance measurement Sub function for Sonar 180 degree check ultrasound
int Distance_test()  {
  float Fdistance=0 ,FFDistance=0;
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS). 
  Fdistance= uS / US_ROUNDTRIP_CM;
  
 if (Fdistance>0)
   {
     FFDistance = Fdistance;
   }
   else
   {
     FFDistance = 60;
  }

 // Serial.print("Fdistance_Test= ");
 // Serial.print(FFDistance);
 // Serial.println();
 // delay(200);
  return FFDistance;
} 

 
//Ultrasonic distance measurement Sub function for Front check ultrasound
int Distance_Front() {
 float Fdistance_Front, FFDistance_Front;
obstacle=false;
 unsigned int uS_Front = sonar_Front.ping(); // Send ping, get ping time in microseconds (uS). 
 Fdistance_Front= uS_Front / US_ROUNDTRIP_CM;
 digitalWrite(laser,HIGH);
 
 if (Fdistance_Front > 0) 
   {
     FFDistance_Front = Fdistance_Front;
     obstacle=true;
   }
 
   else
   {
     FFDistance_Front = 60;
      obstacle=false;
  }
 Serial.print("Distance_Front= ");
 Serial.print(FFDistance_Front);
 Serial.println();
 delay(X);
 digitalWrite(laser,LOW);
 return (int)FFDistance_Front;
}  

void Sonar()         {
         // float iDirection=0;
         // float alfa_i=0;
          int i=0;
          int count=0;
        //  int major=0;
        //  int imax=0; 
        //  int Di=0;
          myservo.write(90); //Angulare between 90-180 RX direction. Angular between 0-90 SX direction 
          delay(X); 
          // routine per azzerare array
             for ( count=0; count<=180; count=count+10)
            {
              sonar_midle_ang[count] = 0;     
            }
       
          //Research Sonar for Free Field avaiable 
                      
            for (i=0; i<=180;i=i+10){
                        myservo.write(i);
                        delay(X);
                        middleDistance=Distance_test();
                        delay(X);
                        middleDistance=abs(middleDistance);
                        sonar_midle_ang[i] = middleDistance;//Array che contiene x= [la distanza] rispetto all'angolo "i" di visuale, se la distanza è maggiore di 60cm ritorna 60 
                        delay(X);
            // Serial.print ("sonar i= ");
              //          Serial.print(i);
                //        Serial.print("  middledistance=");
                  //      Serial.print(middleDistance);
                    //       Serial.println();
                    //    delay(200);
                      
            }
   
}

 int Direction()     {    
  long int count=0;
   long int i,Di,D;
   float iDirection=0;
   float alfa_i=0; 
   Sonar();
          // routine per simulare una distanza fissa
           for ( count=0; count<=180; count=count+10)
            {
             // Serial.println(sonar_midle_ang[count]);     
            }
 
  i=0;
  Di=0;
  D=0;
      for (i=0; i<=180;i=i+10)
            {
                        
                        Di=sonar_midle_ang[i]+Di;
                      //  Serial.println();
                      // Serial.print(" i = ");
                      // Serial.print(i);
                      //  Serial.println();
                      // Serial.print(" Di = ");
                      // Serial.print(Di);
            }
D=Di;
i=0;
Di=0;
alfa_i=0;
      for (i=0; i<=180;i=i+10)
            {
                        Di=sonar_midle_ang[i];
                        if ( i==0) alfa_i=(Di*1)+alfa_i;
                        if (i!=0) alfa_i=(Di*i)+alfa_i;
                        
                      //alfa_i=(Di*i)+alfa_i;
            }
                  //     delay(200);

                      
                  //      Serial.println();
                  //      Serial.print(" alfa_i = ");
                  //      Serial.print(alfa_i);
                   
            
                        
                  //     Serial.println();
                  //     Serial.print(" D TOT= ");
                  //     Serial.print(D);
                  //      Serial.println();

                       Di=alfa_i/D;
                      Serial.print(" Angolo Direzione = ");
                        Serial.print(Di);
                         Serial.println();
                      //  delay(200);
                        iDirection=Di;
                       // Serial.println();
                       // Serial.print(" Sonar iDirection = alfa TOT/Di TOT ");
                       // Serial.print(iDirection);
        delay(X);
                     
       return iDirection;   //direzione da prendere per evitare ostacolo  
}

void setup()         { 
  pinMode(laser,OUTPUT);
  myservo.attach(3,700,2400);  // attach servo on pin 3 to servo object
  myservo.write(90);
  Serial.begin(9600);      
 // Direction();
 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
 
   myservo.write(90);//setservo position according to scaled value
  // frontDistance  = Distance_Front();
  // middleDistance= Distance_test();


} 

void loop()          

{ 
  myservo.write(90);
  Distance_Front();
  //forward(-20);
  //delay(X);
   int i=0;
  
       if (Distance_Front() < 60 || obstacle){
                  stop();
     
              newDirection= Direction();//Direction() restituisce l'angolo dove il sonar ha rilevato la direzione da prendere piu libera 
               
                if ( (newDirection <90) && (newDirection>0)){
                    // la macchina deve girare a Dx dell'angolo corrispondente a newDirection
                   i=0;
                    for (i=0;i<=newDirection;i=i+10){
                       right10(-75);
                       delay (X);
                    }
                   
                  }
                    else if ( (newDirection >90) && (newDirection <180)){
                        i=0;
                    for (i=0;i<=newDirection;i=i+10){
                       left10(-75);
                       delay (X);
                    }
                      }
                        else if ( newDirection==0 || obstacle   ) {
                            back(-50);
                            delay(X+X+X); 
                            left(-45); 
                            delay(X+X+X); 
                          }
                          delay(X);
                          }
     forward(-25);
}
        
      
                    
