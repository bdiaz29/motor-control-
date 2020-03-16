#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


//assign the pinouts
int StepInput=13;
int DIR=14;
//assing the minimum delay after each step for the stepper motor
//in microseconds
const int MinStepDelay=1000;

//serv0 stuff
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
// our servo # counter
uint8_t servonum = 0;
//assing servo feedback pins
int servoFeedbackPin[]={A0,A1,A2,A3};
//the array to store those values 
int servoFeedback[4];
//the array to keep track of position
int servoPosition[4];
//the measured values of the min and max positions
int feedBackMin=0;
int feedBackMax=1023;

//tolerance for servo feedback error
int servoTolerance=2;




void setup() {
  // initialize the digital pins
  pinMode(StepInput, OUTPUT);
  pinMode(DIR, OUTPUT);
  
  //initialize pins to be low 
  digitalWrite(StepInput, LOW);
  digitalWrite(DIR, LOW);
  
   pwm.begin();
   pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
   delay(10);
  
}

//main code
void loop() {
  
  
}

//does N number of steps in the direction specified with  is a boolean 0 or 1
//it then returns a boolean true when it is done operating.
boolean NSteps(int N,boolean Direction,int StepDelay)
{
  boolean done=0;
  //set the direction 
  pinMode(DIR,Direction);
  //run through the steps
  for (int i = 0; i <= N; i++) 
    {
      //brings the StepInput to high triggering the positive edge of easydriver
      digitalWrite(StepInput, HIGH);
      //waits the minimum amount 
      delayMicroseconds(MinStepDelay);
      //waits the inputted delay
      delay(StepDelay);
      //brings the StepInput to low 
      digitalWrite(StepInput, LOW);
    }
  //returns boolean true when it is done
  return done;
}

//does N degree turn in the specified direction the direction specified with  is a boolean 0 or 1
//it then returns a boolean true when it is done operating.
boolean NdegreeTurn(int N,boolean Direction,int StepDelay)
{
  boolean done=0;
  //calculate the number of steps
  //the assumptions is each step makes a 1.8 degree turn
  //first calculate the floating point value
  float s=N/1.8;
  //then is cast into the interger "steps"
  //floating point is truncated not rounded
  int steps=(int)s;
  //calls the NSteps function
  done=NSteps(steps,Direction,StepDelay);
  return done;
}
//for testing purposes does one full revolution at specified direction 
//with specified step delay
 void fullRevo(boolean Direction,int StepDelay)
{
  NdegreeTurn(360,Direction,StepDelay);
  //NSteps(200,Direction,StepDelay);
}


//updates the values of the feedback pins
void updateFeedback()
{
  for(int i=0;i<4;i++)
  {
    //reads the input from the analog input 
    servoFeedback[i]=analogRead(servoFeedbackPin[i]);
    //converts it into degrees
    servoPosition[i]=map(servoFeedback[i],feedBackMin,feedBackMax,0,180);
  }
}


//returns current position of specified servo
int ServoPositionN(int N)
{
  //update the feedback of all the servos
  updateFeedback();
  //assing value to pos
  int  pos=servoPosition[N];
  return pos;
}



//sets the specified servo to the specified position
//returns true if it has acheive this
boolean setServo(int servoNum,int targetPosition)
{ 
 int pulseLength;
 //convert to pulse length
 pulseLength=map(targetPosition,0,180,SERVOMIN,SERVOMAX);
 //set the pwm for the specified servo
 pwm.setPWM(servoNum, 0, pulseLength);
 //wait for the servo to get into position
 //check every 1 millisecond if the servo is within tolerance
 //if it is break out of the for loop
 //this would last a max of 3 seconds if it is not able to get within tolerance
for(int t=0;t<3000;t++)
{
  if(withinTolerance(servoPosition[servoNum],targetPosition))
  break;
  else
  delay(1);
}
//wait half a second to see if there is any settling
delay(500);
 if(withinTolerance(servoPosition[servoNum],targetPosition))
 return true;
 else
 return false;

}






//move all motors simultanously
boolean simult(int servoTarget[4], int stepper,int Direction,int StepDelay)
{ 
  boolean individualServo[]={0,0,0,0};
  boolean sucess;
  boolean servosWithinTolerance[4];
  int pulseLength[4];
  //calculate the number of steps
  //the assumptions is each step makes a 1.8 degree turn
  //first calculate the floating point value
  float s=stepper/1.8;
  //then is cast into the interger "steps"
  //floating point is truncated not rounded
  int steps=(int)s;
  //calculates the pulse lengths
  //and then set eash servo to it 
  for(int j=0;j<4;j++)
  {
     pulseLength[j]=map(servoTarget[j],0,180,SERVOMIN,SERVOMAX);
     pwm.setPWM(j, 0, pulseLength[j]);
  }
  //runs the stepper
  //set the direction 
  pinMode(DIR,Direction);
  //run through the steps
  for (int i = 0; i <= stepper; i++) 
    {
      //if no steps needed break out of foor loop before any changes
      if(stepper==0)
      break;
      //brings the StepInput to high triggering the positive edge of easydriver
      digitalWrite(StepInput, HIGH);
      //waits the minimum amount 
      delayMicroseconds(MinStepDelay);
      //waits the inputted delay
      delay(StepDelay);
      //brings the StepInput to low 
      digitalWrite(StepInput, LOW);
    }
     for (int f=0;f<3000;f++)
     {
      individualServo[0]=withinTolerance(servoPosition[0],servoTarget[0]);
      individualServo[1]=withinTolerance(servoPosition[1],servoTarget[1]);
      individualServo[2]=withinTolerance(servoPosition[2],servoTarget[2]);
      individualServo[3]=withinTolerance(servoPosition[3],servoTarget[3]);
      sucess=individualServo[0]&&individualServo[1]&&individualServo[2]&&individualServo[3];
      if (sucess)
      break;
      else
      delay(1);
     }

    delay(500);
    if (sucess)
      return true;
      else
      return false;
}






//checks if within tolerance
bool withinTolerance(int currentPosition, int targetPosition )
{
  int tolerance=servoTolerance;
  //updates the positions 
  updateFeedback();
  if(
  ((currentPosition-tolerance)>targetPosition)&&
  ((currentPosition+tolerance)<targetPosition)
   )
   return true;
   else
   return false;
}