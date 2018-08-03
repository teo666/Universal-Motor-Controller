/**********************************************************************************************
 * Arduino PID Async Library - Version 1.0.0
 * by teo Basili <basili.teo@gmail.com> https://github.com/teo666
 *
 * this library is based on Brett Beauregard's PID library
 * <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_ASYNC.h"

PID::PID(double *Input, double *Output, double *Setpoint, uint8_t POn,
         CoefficientPtr (*searchfun)(), volatile uint16_t *TimeInterval,
         uint8_t ControllerDirection) {
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  timeInterval = TimeInterval;
  searchfunPtr = searchfun;

  SetTunings(POn);

  PID::SetOutputLimits(0, 255);

 // PID::SetControllerDirection(ControllerDirection);
}

bool PID::Compute() {
  if (!inAuto) {
    return false;
  }

  // it has to find the correct parameters
  CoefficientPtr KS = searchfunPtr();

  /*Compute all the working error variables*/
  double input = *myInput;
  double error = *mySetpoint - input;
  double dInput = (input - lastInput);
  outputSum += (KS->Ki * (*timeInterval) * error);

  /*Add Proportional on Measurement, if P_ON_M is specified*/
  if (!pOnE)
    outputSum -= KS->Kp * dInput;

  if (outputSum > outMax)
    outputSum = outMax;
  else if (outputSum < outMin)
    outputSum = outMin;

  /*Add Proportional on Error, if P_ON_E is specified*/
  double output;
  if (pOnE)
    output = KS->Kp * error;
  else
    output = 0;

  /*Compute Rest of PID Output*/
  output += outputSum - (KS->Kd / (*timeInterval) * dInput);

  if (output > outMax)
    output = outMax;
  else if (output < outMin)
    output = outMin;
  *myOutput = output;

  /*Remember some variables for next time*/
  lastInput = input;

  return true;
}

void PID::SetTunings(/*double Kp, double Ki, double Kd,*/ int POn) {

  //SetTunings(Kp, Ki, Kd);
  pOn = POn;
  pOnE = POn == P_ON_E;
}

/*void PID::SetTunings(double Kp, double Ki, double Kd) {
  if (Kp < 0 || Ki < 0 || Kd < 0)
    return;

  if (controllerDirection == REVERSE) {
    kp = -Kp;
    ki = -Ki;
    kd = -Kd;
  } else {
    kp = Kp;
    ki = Ki;
    kd = Kd;
  }
}*/

void PID::SetOutputLimits(double Min, double Max) {
  if (Min >= Max)
    return;
  outMin = Min;
  outMax = Max;

  if (inAuto) {
    if (*myOutput > outMax)
      *myOutput = outMax;
    else if (*myOutput < outMin)
      *myOutput = outMin;

    if (outputSum > outMax)
      outputSum = outMax;
    else if (outputSum < outMin)
      outputSum = outMin;
  }
}

void PID::SetMode(int Mode) {
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto) { /*we just went from manual to auto*/
    PID::Initialize();
  }
  inAuto = newAuto;
}

void PID::Initialize() {
  outputSum = *myOutput;
  lastInput = *myInput;
  if (outputSum > outMax)
    outputSum = outMax;
  else if (outputSum < outMin)
    outputSum = outMin;
}

/*void PID::SetControllerDirection(int Direction) {
  if (inAuto && Direction != controllerDirection) {
    kp = -kp;
    ki = -ki;
    kd = -kd;
  }
  controllerDirection = Direction;
}*/

/*double PID::GetKp() { return kp; }
double PID::GetKi() { return ki; }
double PID::GetKd() { return kd; }
int PID::GetMode() { return inAuto ? AUTOMATIC : MANUAL; }
int PID::GetDirection() { return controllerDirection; }*/
