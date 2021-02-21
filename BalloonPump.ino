/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  the pid is designed to Output an analog value,
 * but the relay can only be On/Off.
 *
 *   to connect them together we use "time proportioning
 * control"  it's essentially a really slow version of PWM.
 * first we decide on a window size (5000mS say.) we then
 * set the pid to adjust its output between 0 and that window
 * size.  lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the
 * window being "Relay Off Time"
 ********************************************************/
#include <Arduino.h>
#include <Wire.h>

//#include <BMx280I2C.h>
#include <PID_v2.h>
#include "BMP280_TempPres.h"


#define PIN_INPUT 0
#define RELAY_PIN 6
#define ArraySize 3

float Times[ArraySize] = {.5,.75,1.};
float setPoint[ArraySize] = {200,600,.900};
float startPoint = 0.;
//Define Variables we'll be connecting to
float setpoint;
#include "command_buffer.h"
//CCommandBuffer commandBuffer;
#include "commandProcessing.h"

//Specify the links and initial tuning parameters
double Kp=10, Ki=.01, Kd=0;
PID_v2 myPID( Kp, Ki, Kd, PID::Direct);

float pmap(float p, float lowi, float highi, float lowo, float higho)
{
float remap = p*(higho-lowo)/(highi-lowi);
return remap;
}


double setpt;
int i = 1;
float setPointFunc()
{
  //return 10.;
  if (i > ArraySize)  return setPoint[ArraySize];
  float timeMinute = (float)(millis()/(60*100))/10.;
  Serial.print(" Time ");
  Serial.println(timeMinute);

  if (timeMinute > Times[i])
  {
    return setPoint[i];
    i++;
  }
  else
  {
    return setPoint[i-1];
  }

  return startPoint;
}

int WindowSize = 1000;
unsigned long windowStartTime;

void setup()
{
  Serial.begin(9600);
  //Serial.println(" Setup start ");
  presSetup();
  setpoint = .45;
  pinMode(RELAY_PIN, OUTPUT);
   pinMode(LED_BUILTIN, OUTPUT);

  //initialize the variables we're linked to
  //Setpoint = setPointFunc();
  setpt = pmap(setpoint, 0, .60, 0, 1000);
  //tell the PID to range between 0 and the full window size
  //Serial.println(" PID start ");
  myPID.SetOutputLimits(0, WindowSize);
  myPID.Start(0,            // current input
              0,         // current output
              setpt);         // setpoint

  windowStartTime = millis();
  //Serial.println(" Begin Loop ");

}
int iCount = 0;
void loop()
{
  //Serial.println(" Read Serial ");
    //
  // command line interface
  //
  if( Serial.available() )
  {
    int inByte = Serial.read();
    commandBuffer.handleInputChar( inByte );

    if( commandBuffer.endsWith("\r") || commandBuffer.endsWith("\n") )
    {
      processTerminalCommands();
      commandBuffer.reset();
    }
  }
  //delay(500);
  //Setpoint = setPointFunc();
  //Input = analogRead(PIN_INPUT);
  //Input = pressure();
  
  float press = pressure();

/*   if (++iCount % 10 == 0)
  {
    Serial.print(iCount);
    Serial.print(" pressure ");
    Serial.println(press);
  } */

  const double input = pmap(press, 0, .60, 0, 1000);
  
  const double output = myPID.Run(input);
    Serial.print(setpoint*100.);
    Serial.print("\t");
    Serial.println(press*100.);
  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  while (millis() - windowStartTime > WindowSize) {
    // time to shift the Relay Window

    windowStartTime += WindowSize;
  }
  float diff = (float) (millis() - windowStartTime);
    // if (iCount % 10 == 0)
    // {
    //   Serial.print(" out ");
    //   Serial.print(output);
    //   Serial.print(" diff ");
    //   Serial.println(diff);
    // }

  if (output > diff)
    {digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);}
  else
    {digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);}
}

