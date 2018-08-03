#ifndef PID_ASYNC
#define PID_ASYNC
#define LIBRARY_VERSION 1.0.0

typedef struct _coefficient {
  double Kp;
  double Ki;
  double Kd;
} Coefficient;

typedef Coefficient *CoefficientPtr;

class PID {

public:
// Constants used in some of the functions below
#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1
#define P_ON_M 0
#define P_ON_E 1

  // commonly used functions
  // **************************************************************************

  /**
   * constructor
   */
  PID(double *Input, double *Output, double *Setpoint, uint8_t POn,
      CoefficientPtr (*searchfun)(),
      volatile uint16_t *TimeInterval, uint8_t ControllerDirection);

  /**
   * sets PID to either Manual (0) or Auto (non-0)
   */
  void SetMode(int Mode);

  /**
   *
   * performs the PID calculation.  it should be
   * called every time loop() cycles. ON/OFF and
   * calculation frequency can be set using SetMode
   * SetSampleTime respectively
   */
  bool Compute();
  /**
   * clamps the output to a specific range. 0-255 by default, but
   * it's likely the user will want to change this depending on
   * the application
   */
  void SetOutputLimits(double, double);

  // available but not commonly used functions
  // ********************************************************
 // void SetTunings(
  //    double, double, // * While most users will set the tunings once in the
  //    double);        //   constructor, this function gives the user the option
               //   of changing tunings during runtime for Adaptive control
  void SetTunings(int);

 ///void SetControllerDirection(
      //int); // * Sets the Direction, or "Action" of the controller. DIRECT
            //   means the output will increase when error is positive. REVERSE
            //   means the opposite.  it's very unlikely that this will be
            //   needed once it is set in the constructor.
//void
 // SetSampleTime(int); // * sets the frequency, in Milliseconds, with which
                      //   the PID calculation is performed.  default is 100

  // Display functions
  // ****************************************************************
  //double GetKp();     // These functions query the pid for interal values.
 //// double GetKi();     //  they were created mainly for the pid front-end,
 // double GetKd();     // where it's important to know what is actually
 // int GetMode();      //  inside the PID.
 // int GetDirection(); //

private:
  void Initialize();

 // double kp; // * (P)roportional Tuning Parameter
 // double ki; // * (I)ntegral Tuning Parameter
 // double kd; // * (D)erivative Tuning Parameter

  //int controllerDirection;
  int pOn;

  double *myInput;  // * Pointers to the Input, Output, and Setpoint variables
  double *myOutput; //   This creates a hard link between the variables and the
  double
      *mySetpoint; //   PID, freeing the user from having to constantly tell us
                   //   what these values are.  with pointers we'll just know.

  // unsigned long lastTime;
  double outputSum, lastInput;
  volatile uint16_t *timeInterval;

  CoefficientPtr (*searchfunPtr)();
  // unsigned long SampleTime;
  double outMin, outMax;
  bool inAuto, pOnE;
};
#endif
