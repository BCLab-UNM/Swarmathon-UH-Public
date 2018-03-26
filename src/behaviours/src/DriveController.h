#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H

#include "PID.h"
#include "Controller.h"
#include <angles/angles.h>

class DriveController : virtual Controller
{
public:
  DriveController();
  ~DriveController();

  void Reset() override;
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  void SetResultData(Result result) {this->result = result;}
  void SetVelocityData(float linearVelocity,float angularVelocity);
  void SetCurrentLocation(Point currentLocation) {this->currentLocation = currentLocation;}

private:

  time_t waitTime;//timer for delays
  time_t elapsedTime;
  int waitLength = 1;//seoncds to wait betwen each movement

  int rotateTimeCnt;//counter
  int rotateTimeAmt = 5;//how many pulses to pause for hang5 timer

  int rotateDirSwitchCount; //for counting the amount of times the rotate direction switches in one STATE_MACHINE_ROTATE
  float errorYaw;
  float errorYawPrev;
  int rotateCntTol = 3; //how many times rotate dirrection can change before too many

  int rotateTimeTol = 15; //minimum time swarmie can rotate before it goes slower
  int rotateScale = 1; //used to slow PID speed, 1 is normal, 2 means half speed, etc.

  Result result;

  //MAX PWM is 255
  //abridge currently limits MAX to 120 to prevent overcurrent draw
  float left; //left wheels PWM value
  float right; //right wheels PWM value

  bool interupt = false; //hold if interupt has occured yet

  float rotateOnlyAngleTolerance = .15;//.005  //May be too low?
  float finalRotationTolerance = 0.1; //dead code not used
  const float waypointTolerance = 0.15; //15 cm tolerance.

  //0.65 MAX value
  float searchVelocity = 0.35; // meters/second

  float linearVelocity = 0;
  float angularVelocity = 0;

  // Numeric Variables for rover positioning
  Point currentLocation;
  Point currentLocationMap;
  Point currentLocationAverage;

  Point centerLocation;
  Point centerLocationMap;
  Point centerLocationOdom;

  vector<Point> waypoints;

  //PID configs************************
  PIDConfig fastVelConfig();
  PIDConfig fastYawConfig();
  PIDConfig slowVelConfig();
  PIDConfig slowYawConfig();
  PIDConfig constVelConfig();
  PIDConfig constYawConfig();

  void fastPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw, int scale);
  void slowPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw);
  void constPID(float erroVel,float constAngularError, float setPointVel, float setPointYaw);

  //each PID movement paradigm needs at minimum two PIDs to acheive good robot motion.
  //one PID is for linear movement and the second for rotational movements
  PID fastVelPID;
  PID fastYawPID;

  PID slowVelPID;
  PID slowYawPID;

  PID constVelPID;
  PID constYawPID;

  // state machine states
  enum StateMachineStates {

    //WAITING should not be handled- goes to default (it's a placeholder name)
    STATE_MACHINE_WAITING = 0,
    STATE_MACHINE_PRECISION_DRIVING,
    STATE_MACHINE_WAYPOINTS,
    STATE_MACHINE_ROTATE,
    STATE_MACHINE_SKID_STEER,
    STATE_MACHINE_HANG_10
  };


  StateMachineStates stateMachineState = STATE_MACHINE_WAITING;
  StateMachineStates savedState; //for returning from hang10 wait
  void ProcessData();

};

#endif // DRIVECONTROLLER_H
