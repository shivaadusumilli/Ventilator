//use THIS as Debugging

/*
bpm = 8-40
volume = 200-800
i/e = 1:1 - 1:4

MAX CASE:
40*5*60 degrees per min
12000 degrees per min

12000 - 33
0 - 0


motor:
33RPM
1 min = 33*360 degrees
11880  degrees per min
1 sec = 60*33 degrees


SPEED     RPM
100       33
50        16.5
0         0

60 - 800
0 - 0
1 - 13.3
*/


#include "Wire.h"
#include "LiquidCrystal_I2C.h"
//Medical Ranges
long int TotalVolume=1600; // of bag;
long int BPM_MIN= 8;
long int BPM_MAX=41;
long int IE_MIN=1;
long int IE_MAX=5;
long int Volume_MIN=200;
long int Volume_MAX=810;

//Hardware Ranges
long int Volume0Angle=60;
long int VolumeFullAngle=0;

long int Feedback0 = 140;
long int Feedback90 = 350;


//pins
int M1=11; // motor pin1
int M2=12; // motor pin2
int FP=A3; // motor Feedback pin
int ST=2;  // Start/Stop pin
int BP=A2; // BPM nob pin
int IP=A0; // IE nob pin
int VP=A1; // Volume nob pin
int EP=10; //enable for motor
int BA=A6; //Battery voltage pin
int SB=5;  //soundbuzer;


//Variables
long int BPM=5;
long int IE=1;
long int Volume=800;


// system variables
bool systemState=0;
String L1=" SOLBOTS VENTILATOR ";
String L2 = "BPM = "+String(BPM)+"  ";
String L3 = "Volume = "+String(Volume)+"  ";
String L4 = "IE Ratio = 1:"+String(IE)+"  ";



bool Debugging=false;
long int lastPowerPress = 0;
LiquidCrystal_I2C  lcd(0x27,20,4);

void setup(){
  Serial.begin(9600);
  pinMode(M1, OUTPUT);  
  pinMode(M2, OUTPUT);
  pinMode(FP, INPUT);
  pinMode(BP, INPUT);
  pinMode(IP, INPUT);
  pinMode(VP, INPUT);
  pinMode(ST, INPUT_PULLUP);
  pinMode(SB, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ST), UpdateSystemState , RISING );
  lcd.begin();
}

long int INTERRUPT()
{
    long int newBPM = BPM_MIN + (((float)analogRead(BP))/1024.0)*((float)BPM_MAX - BPM_MIN);
    if(newBPM != BPM)
    {
      BPM = newBPM;
      Serial.println("BPM updated, new BPM = "+String(BPM));    
      L2 = "BPM = "+String(BPM)+"   ";
    UpdateScreen();
    }


    long int newIE = IE_MIN + ((float)analogRead(IP)/1024.0)*(IE_MAX - IE_MIN);
    if(IE != newIE)
    {
      IE = newIE;
      Serial.println("IE updated, new IE = 1:"+String(IE)+"   ");
          L4 = "IE Ratio = 1:"+String(IE)+"   ";
    UpdateScreen();
    }

   long int newVolume = Volume_MIN + ((float)analogRead(VP)/1024.0)*(Volume_MAX - Volume_MIN);
   newVolume = newVolume - newVolume%10;
   if(Volume != newVolume)
    {
      Volume = newVolume;
      Serial.println("Volume updated, new Volume = "+String(Volume)+"   ");
          L3 = "Volume = "+String(Volume)+" ML";
    UpdateScreen();
    }
}

void UpdateScreen()
{
  lcd.setCursor(0, 0);
  lcd.print(L1);
  lcd.setCursor(0, 1);
  lcd.print(L2);
  lcd.setCursor(0, 2);
  lcd.print(L3);
  lcd.setCursor(0, 3);
  lcd.print(L4);
  }

long int getAngle()
{
return ((float)(analogRead(FP)-Feedback0)/(float)(Feedback90 - Feedback0))*90.0;
}

long int getVolume()
{
  //Update this
return ((float)(Volume0Angle - (90-getAngle()))/(float)(Volume0Angle))*(Volume_MAX - Volume_MIN);
}

bool Motor(int SPEED,int VOLUME,int IE,long int ST)
{
Serial.println("IN MOTOR");
Serial.println("Speed = "+String(SPEED)+" VOLUME = "+String(VOLUME));
analogWrite(EP,SPEED*2.55);
digitalWrite(M1,1);
digitalWrite(M2,0);
while(getVolume() <= VOLUME)
{
  Serial.println("CV="+String(getVolume())+" "+"RV="+String(VOLUME)+" "+"A:"+String(getAngle())+"  ");
  delay(50);
  //INTERRUPT();
 }
analogWrite(EP,100);  
while((millis() - ST ) < TotalTime(BPM)/(float(IE+1)))
{
Serial.println("waiting for expiration");
delay(10);
}
 
digitalWrite(M1,0);
digitalWrite(M2,1);
analogWrite(EP,(SPEED*2.55));  
while(getVolume() > 0)
{
    Serial.println("CV="+String(getVolume())+" "+"RV=0 "+"A:"+String(getAngle())+" ");
    delay(50);
    //INTERRUPT();
}
    digitalWrite(M1,1);
  digitalWrite(M2,0);
  delay(20);
  analogWrite(EP,0);
}

void UpdateSystemState()
{
  if(millis()/1000 == lastPowerPress)
  return;
  lastPowerPress = millis()/1000;
  if(systemState)
  systemState=0;
  else
  systemState=1;
  Serial.println("System state updated");
 }
 /*
void UpdateBPM()
{
  
  long int newBPM = BPM_MIN + (((float)analogRead(BP))/1024.0)*((float)BPM_MAX - BPM_MIN);
  if(BPM != newBPM)
  {
    BPM = newBPM;
    Serial.println("BPM updated, new BPM = "+String(BPM));
    L2 = "BPM = "+String(BPM)+"  ";
    UpdateScreen();
    }
  }

void UpdateIE()
{
  long int newIE = IE_MIN + ((float)analogRead(IP)/1024.0)*(IE_MAX - IE_MIN);
  if(IE != newIE)
  {
    IE = newIE;
    Serial.println("IE updated, new IE = 1:"+String(IE));
    L4 = "IE Ratio = 1:"+String(IE);
    UpdateScreen();
    }
  }

void UpdateVolume()
{
  long int newVolume = Volume_MIN + ((float)analogRead(VP)/1024.0)*(Volume_MAX - Volume_MIN);
  if(Volume != newVolume)
  {
    Volume = newVolume;
    Serial.println("Volume updated, new Volume = "+String(Volume));
    L3 = "Volume = "+String(Volume);
    UpdateScreen();
    }
}
*/
long int getSpeed(long int BPM,long int volume, long int IERATIO) //Range of speed = [0-100]
{
  Serial.println("Inputs to speed = "+String(BPM)+","+String(volume)+","+String(IERATIO));
  long int SPEED = (BPM * volume * (IERATIO+1))/1600;
  Serial.println("Returning Speed = "+String(SPEED));
  //delay(4000);
  return 100;
  return SPEED;
}
long int TotalTime(int BPM)
{
  long int TT=(60000/BPM);
  Serial.println("Returning time = "+String(TT)+" for bpm = "+String(BPM));
  return TT;
  }
 void Buzzer()
 {
  digitalWrite(SB,HIGH);
  delay(100);
  digitalWrite(SB,LOW);  
  }
void loop() {
  UpdateScreen();
  if(systemState)
  {
    INTERRUPT();
    long int ST = millis();
    Motor(getSpeed(BPM,Volume,IE),Volume,IE,ST);
    if((millis() - ST ) > TotalTime(BPM))
        {
          Serial.println("ERROR");
          Buzzer();
        }
    else
    {
      Serial.println("Sleeping for "+String(TotalTime(BPM) - (millis() - ST))+" milli seconds");
      while((millis() - ST ) < TotalTime(BPM))
      {
     INTERRUPT();
     delay(10);
        }
    }
  }
  else
  {
  INTERRUPT();
  Serial.println("System state : False");
  Serial.println(analogRead(FP));
  }
}
