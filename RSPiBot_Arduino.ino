#define MAX_DATA_SIZE 10

#define LOOKUP_TABLE_SIZE 11
#define AVERAGE_SAMPLES 10
double lookUp[LOOKUP_TABLE_SIZE][2];

unsigned long time;
unsigned long setTime;
char incommingData[MAX_DATA_SIZE] = {0};

int mA1 = 5;
int mA2 = 6;
int mB1 = 9;
int mB2 = 10;

unsigned char speedA = 0;
boolean forwardA = true;

unsigned char speedB = 0;
boolean forwardB = true;
int serialCounter = 0;

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

double sensorValue = 0;
double distence =0;
int i = 0;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  pinMode(mA1, OUTPUT);
  pinMode(mA2, OUTPUT);
  pinMode(mB1, OUTPUT);
  pinMode(mB2, OUTPUT);
  speedA = 0;
  speedB = 0;
  setTime = millis();

    if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /*init the lookup table */
  lookUp[0][0] = 630;
  lookUp[1][0] = 533;
  lookUp[2][0] = 407;
  lookUp[3][0] = 300;
  lookUp[4][0] = 190;
  lookUp[5][0] = 157;
  lookUp[6][0] = 88;
  lookUp[7][0] = 58;
  lookUp[8][0] = 36;
  lookUp[9][0] = 30;
  lookUp[10][0] = 18;
  
  lookUp[0][1] = 1;
  lookUp[1][1] = 2;
  lookUp[2][1] = 4;
  lookUp[3][1] = 6;
  lookUp[4][1] = 11;
  lookUp[5][1] = 14;
  lookUp[6][1] = 26;
  lookUp[7][1] = 38;
  lookUp[8][1] = 54;
  lookUp[9][1] = 75;
  lookUp[10][1] = 80;
  
}

void loop() 
{
 
  if (Serial.available() > 3)
  {
    serialCounter = 0;
    while (serialCounter<MAX_DATA_SIZE-1 )
    {
      while (!Serial.available());
      incommingData[serialCounter] = Serial.read();
      //end of command so break

      if (incommingData[serialCounter] == 'S')
    {
        break;
    }
      serialCounter++;
      if (incommingData[0] < 'A')
        serialCounter = 0;
    }
  if (incommingData[0] == 'D')
  {
    //data is motor speed data
    int driveSpeed = 0;
    boolean forward = false;
 
    if(serialCounter == 4)
    {
      driveSpeed = (incommingData[2]-48)*1;
      if (incommingData[3] == 'F') 
        forward = true;
    }
    if(serialCounter == 5)
    {
      driveSpeed = (incommingData[2]-48)*10;
      driveSpeed += (incommingData[3]-48)*1;
      if (incommingData[4] == 'F') 
        forward = true;
    }
    if(serialCounter == 6)
    {
      driveSpeed = (incommingData[2]-48)*100;
      driveSpeed += (incommingData[3]-48)*10;
      driveSpeed += (incommingData[4]-48)*1;
      if (incommingData[5] == 'F') 
        forward = true;
    }
    if (incommingData[1] == 'L')
    {
      speedA = driveSpeed;
      forwardA = forward;
      setTime = millis();
    }
    else
    {
      speedB = driveSpeed;
      forwardB = forward;
      setTime = millis();
    }
  }
  }

   time = millis();
   if ((time - setTime) > 2000)
   {
     //setTime = millis();
     speedA /= 2;
     speedB /= 2;
   }
   
   if (speedA < 50)
     speedA = 0;
  
  if (speedB < 50)
   speedB = 0;
   
  if (forwardA)
  {
     analogWrite(mA1, speedA);
     analogWrite(mA2, 0);
  }
  else
  {
     analogWrite(mA1, 0);
     analogWrite(mA2, speedA);
  }
  
  if (forwardB)
  {
     analogWrite(mB1, speedB);
     analogWrite(mB2, 0);
  }
  else
  {
     analogWrite(mB1, 0);
     analogWrite(mB2, speedB);
  }
  
   /*compass code */
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  float cx = event.magnetic.x;
  float cy = event.magnetic.y;
  float cz = event.magnetic.z;

  /*******************************************************************************/
  /* distence sensor */
  
  sensorValue = 0;
  
  for (i = 0; i<AVERAGE_SAMPLES; i ++)
  {
   sensorValue += analogRead(A0);
   delay(2); //make it readable
  }
  sensorValue /= AVERAGE_SAMPLES;
  
  
  if (sensorValue > lookUp[0][0])
  {
    distence = lookUp[0][1];
  }
  else if (sensorValue < lookUp[LOOKUP_TABLE_SIZE-1][0])
  {
    distence = lookUp[LOOKUP_TABLE_SIZE-1][1];
  }
  else
  {
    for (i = 0; i< LOOKUP_TABLE_SIZE; i++)
    {
       if (sensorValue > lookUp[i][0])
       {
         double y1 = lookUp[i-1][1];
         double x1 = lookUp[i-1][0];
         
         if (1+1 < LOOKUP_TABLE_SIZE)
           i++;
         
         double y2 = lookUp[i][1];
         double x2 = lookUp[i][0];
         
         double x = sensorValue;
         
         distence = y1+ ( ( (x-x1)*(y2-y1) )/(x2-x1));
         break;
       }
    }
  }
  
  /* output */
  Serial.print ("FB/CXYZ="); Serial.print(cx); Serial.print("*"); Serial.print(cy); Serial.print("*"); Serial.print(cz); Serial.print("/D="); Serial.print((int)distence); Serial.println("/LB");
  
}




