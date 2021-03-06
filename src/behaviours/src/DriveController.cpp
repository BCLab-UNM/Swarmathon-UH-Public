#include "DriveController.h"

DriveController::DriveController() {

  fastVelPID.SetConfiguration(fastVelConfig());
  fastYawPID.SetConfiguration(fastYawConfig());

  slowVelPID.SetConfiguration(slowVelConfig());
  slowYawPID.SetConfiguration(slowYawConfig());

  constVelPID.SetConfiguration(constVelConfig());
  constYawPID.SetConfiguration(constYawConfig());


}

DriveController::~DriveController() {}

void DriveController::Reset()
{
  waypoints.clear();

  if (stateMachineState == STATE_MACHINE_ROTATE || stateMachineState == STATE_MACHINE_SKID_STEER)
  {
    stateMachineState = STATE_MACHINE_WAYPOINTS;
  }
}

Result DriveController::DoWork()
{

  ///WARNING waypoint input must use FAST_PID at this point in time failure to set fast pid will result in no movment

  if(result.type == behavior)
  {
    if(result.b == noChange)
    {
      //if drive controller gets a no change command it is allowed to continue its previous action
      //normally this will be to follow waypoints but it is not specified as such.
    }

    else if(result.b == wait)
    {
      //do nothing till told otherwise
      left = 0.0;
      right = 0.0;
      stateMachineState = STATE_MACHINE_WAITING;
    }

  }

  else if(result.type == precisionDriving)
  {

    //interpret input result as a precision driving command
    stateMachineState = STATE_MACHINE_PRECISION_DRIVING;

  }

  else if(result.type == waypoint)
  {
    //interpret input result as new waypoints to add into the queue
    ProcessData();//sets state to STATE_MACHINE_WAYPOINTS

  }
  //either precision driving or waypoints
  //if waypoints, first checks if goal is further than tollerance of current posiont (> 15cm away) or removes point
  //then rotates if angle is > tollerance. if angle doesnt go within tolerance after rotation skips linear movement
  //then goes forward until movement is less than tolerance
  //no break statements if tolerance checks work to auto flow into next case

  switch(stateMachineState)//keeps the states between each call of function
  {

    //Handlers and the final state of STATE_MACHINE are the only parts allowed to call INTERUPT
    //This should be done as little as possible. I suggest using timeouts to set control bools to false.
    //Then only call INTERUPT if bool switches to true.
    case STATE_MACHINE_PRECISION_DRIVING:
    {

      ProcessData();
      break;
    }


    case STATE_MACHINE_WAYPOINTS:
    {
      //cout << "STATE_MACHINE_WAYPOINTS: curLoc = " << currentLocation.x <<"," << currentLocation.y << endl;//DEBUGGING
      //cout << "targetLoc = " << waypoints.back().x <<"," << waypoints.back().y << endl;//DEBUGGING
      //Handles route planning and navigation as well as making sure all waypoints are valid.

      bool tooClose = true;
      //while we have waypoints and they are tooClose to drive to
      while (!waypoints.empty() && tooClose)
      {
        //check next waypoint for distance
        if (hypot(waypoints.back().x-currentLocation.x, waypoints.back().y-currentLocation.y) < waypointTolerance)//tol = 15 cm
        {
          //if too close remove it
          waypoints.pop_back();
        }
        else
        {
          //this waypoint is far enough to be worth driving to
          tooClose = false;
        }
      }

      //if we are out of waypoints then interupt and return to logic controller
      if (waypoints.empty())
      {
        stateMachineState = STATE_MACHINE_WAITING;
        result.type = behavior;
        interupt = true;
        return result;
      }
      else
      {
        //select setpoint for heading and begin driving to the next waypoint
        stateMachineState = STATE_MACHINE_ROTATE;
        waypoints.back().theta = atan2(waypoints.back().y - currentLocation.y, waypoints.back().x - currentLocation.x);//set theta to relative theta to goal
        result.pd.setPointYaw = waypoints.back().theta;

        rotateTimeCnt = 0;//for hang5
        rotateDirSwitchCount = 0;//reset for use in STATE_MACHINE_ROTATE
        waitTime = time(0);//timer for how long swarmie has been rotating
        //cout << "**************************************************************************" << endl; //DEBUGGING CODE
        //cout << "Waypoint x : " << waypoints.back().x << " y : " << waypoints.back().y << endl; //DEBUGGING CODE
        //fall through on purpose
      }
    }

    case STATE_MACHINE_ROTATE:
    {
      //cout << "ROTATING: curLoc = " << currentLocation.x <<"," << currentLocation.y <<endl;//DEBUGGING
      // Calculate angle between currentLocation.theta and waypoints.front().theta
      // Rotate left or right depending on sign of angle
      // Stay in this state until angle is minimized

      waypoints.back().theta = atan2(waypoints.back().y - currentLocation.y, waypoints.back().x - currentLocation.x);

      //Calculate the diffrence between current and desired heading in radians.
      //Swarmie has an issue where if the point is behind it, it sometimes rotates back and forth with errorYaw switching between ~-255 and ~255
      //Counts amount of times swarmie switches and if >rotateCntTol (3) it just rotates right
      /*if(rotateDirSwitchCount < rotateCntTol){
        errorYawPrev = errorYaw; //save previous value for counting dirrection changes
        errorYaw = angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta);
        if ((errorYawPrev > 0 && errorYaw < 0) || (errorYawPrev < 0 && errorYaw > 0)){
          rotateDirSwitchCount++;
        }
      }
      else {*/
        errorYaw = fabs(angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta));
      //}
      //end errorYaw calc

      //timer for counting how long swarmie has been rotating, if longer than rotateTimeTol (15) then will slow down
      /*elapsedTime = time(0); //TODO test to varrify, seems to be working, however one time running the swarmie it just stopped moving but that may be do to something else
      if((elapsedTime - waitTime > rotateTimeTol) && (elapsedTime - waitTime < (rotateTimeTol*3))){//check rotate time
        //if still rotating in place for rotateTimeTol*3 go back to initial speed, catch for if it stops moving
        rotateScale = 2;
      }
      else {//normal speed if it hasn't rotated for long enough or for too long
        rotateScale = 1;
      }*/
      //end timer

      //cout << "ROTATE Error yaw:  " << errorYaw << " target heading: " << waypoints.back().theta << ", current heading: " << currentLocation.theta << endl; //DEBUGGING CODE
      //cout << "Waypoint: " << waypoints.back().x << "," << waypoints.back().y << " currentLoc: " << currentLocation.x << "," << currentLocation.y << endl; //DEBUGGING CODE

      result.pd.setPointVel = 0.0;
      //Calculate absolute value of angle

      float abs_error = fabs(angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta));

      // If angle > rotateOnlyAngleTolerance radians rotate but dont drive forward.
      if (abs_error > rotateOnlyAngleTolerance) //if angle > tolerance then break so no go forward
      {
        // rotate but dont drive.
        if (result.PIDMode == FAST_PID)
        {
          if (rotateTimeCnt < rotateTimeAmt){//run fastPId for #rotateTimeAmt (5) pulses then wait for camera to catch up
            fastPID(0.0, errorYaw, result.pd.setPointVel, result.pd.setPointYaw, 1);
            rotateTimeCnt++;
          }/*else{
            waitTime = time(0);//get current time for hang5 timer
            savedState = STATE_MACHINE_ROTATE;
            stateMachineState = STATE_MACHINE_HANG_10;//to make pause for a little after each movement
            rotateTimeCnt = 0;
          }*/
        }

        break;
      }
      else
      {
        //move to differential drive step
        stateMachineState = STATE_MACHINE_SKID_STEER;
        break;
      }
    }

    case STATE_MACHINE_SKID_STEER: //return to this state until distance < tolerance
    {
        // Calculate angle between currentLocation.x/y and waypoints.back().x/y
        // Drive forward
        // Stay in this state until angle is at least PI/2

      // calculate the distance between current and desired heading in radians
      //cout << "going 4ward: curLoc = " << currentLocation.x <<"," << currentLocation.y <<endl;//DEBUGGING
      waypoints.back().theta = atan2(waypoints.back().y - currentLocation.y, waypoints.back().x - currentLocation.x);
      float errorYaw = angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta);
      float distance = hypot(waypoints.back().x - currentLocation.x, waypoints.back().y - currentLocation.y);

      //cout << "Skid steer, Error yaw:  " << errorYaw << " target heading : " << waypoints.back().theta << " current heading : " << currentLocation.theta << " error distance : " << distance << endl; //DEBUGGING CODE
      //cout << "Waypoint x : " << waypoints.back().x << " y : " << waypoints.back().y << " currentLoc x : " << currentLocation.x << " y : " << currentLocation.y << endl; //DEBUGGING CODE



      // goal not yet reached drive while maintaining proper heading.
      if (fabs(errorYaw) < M_PI_2 &&  distance > waypointTolerance)
      {
        // drive and turn simultaniously
        result.pd.setPointVel = searchVelocity;
        if (result.PIDMode == FAST_PID)
        {
          //cout << "linear velocity:  " << linearVelocity << endl; //DEBUGGING CODE
          fastPID((searchVelocity-linearVelocity) ,errorYaw, result.pd.setPointVel, result.pd.setPointYaw, 1);
        }
      }
      else {
        // stopno change
        waitTime = time(0);//get current time for hang10 timer
        //savedState = STATE_MACHINE_WAYPOINTS;//for HANG_10
        //stateMachineState = STATE_MACHINE_HANG_10;//to make pause for a little after each movement
        stateMachineState = STATE_MACHINE_WAYPOINTS;
      }

      break;
    }

    case STATE_MACHINE_HANG_10://pause after each movement to let camera catch up and prevent blurring
    {//not used, instead pause moved to ROS adapter
      cout << "Hanging 10, return: " << savedState << endl;//DEBUGGING
      //cout << "wait: " << waitTime << ", elapsed: " << elapsedTime <<endl;//DEBUGGING
      right = 0;
      left = 0;

      elapsedTime = time(0);
      if(elapsedTime - waitTime < waitLength){//wait for waitLength seconds
        break;
      }
      else {
        stateMachineState = savedState;
        cout << "done hanging, bro" << endl;//DEBUGGING
        break;
      }
    }

    default:
    {
      break;
    }


  }

  //package data for left and right values into result struct for use in ROSAdapter
  result.pd.right = right;
  result.pd.left = left;

  //return modified struct
  return result;

}

bool DriveController::ShouldInterrupt()
{
  if (interupt)
  {
    interupt = false;
    return true;
  }
  else
  {
    return false;
  }
}

bool DriveController::HasWork() {   }



void DriveController::ProcessData()
{
  //determine if the drive commands are waypoint or precision driving
  if (result.type == waypoint) {

    //sets logic controller into stand by mode while drive controller works
    result.type = behavior;
    result.b = noChange;

    if(result.reset) {
      waypoints.clear();
    }

    //add waypoints onto stack and change state to start following them
    if (!result.wpts.waypoints.empty()) {
      waypoints.insert(waypoints.end(),result.wpts.waypoints.begin(), result.wpts.waypoints.end());
      stateMachineState = STATE_MACHINE_WAYPOINTS;
    }
  }
  else if (result.type == precisionDriving)
  {

    //calculate inputs into the PIDS for precision driving
    if (result.PIDMode == FAST_PID)
    {
      float vel = result.pd.cmdVel -linearVelocity;
      float setVel = result.pd.cmdVel;
      fastPID(vel,result.pd.cmdAngularError, setVel, result.pd.setPointYaw, 1);
    }
    else if (result.PIDMode == SLOW_PID)
    {
      //will take longer to reach the setPoint but has less chanse of an overshoot especially with slow feedback
      float vel = result.pd.cmdVel -linearVelocity;
      float setVel = result.pd.cmdVel;
      slowPID(vel,result.pd.cmdAngularError, setVel, result.pd.setPointYaw);
    }
    else if (result.PIDMode == CONST_PID)
    {
      //vel is the same as fast PID however
      //this setup takes a target angular velocity to constantly turn at instead of a target heading
      float vel = result.pd.cmdVel - linearVelocity;
      float angular = result.pd.cmdAngular - angularVelocity;

      //cout << "Ang. Vel.  " << angularVelocity << "  Ang. Error" << angular << endl; //DEBUGGING CODE

      constPID(vel, angular ,result.pd.setPointVel, result.pd.setPointYaw);
    }
  }
}


void DriveController::fastPID(float errorVel, float errorYaw , float setPointVel, float setPointYaw, int scale)
{//TODO tested and seems to be working but might be messing up the distances. ie. does .5m at 1 but only .25m at 2

  // cout << "PID FAST" << endl; //DEBUGGING CODE

  float velOut = fastVelPID.PIDOut(errorVel, setPointVel); //returns PWM target to try and get error vel to 0
  float yawOut = fastYawPID.PIDOut(errorYaw, setPointYaw); //returns PWM target to try and get yaw error to 0

  int left = velOut - yawOut; //combine yaw and vel PWM values
  int right = velOut + yawOut; //left and right are the same for vel output but opposite for yaw output
  //cout << "Left: " << left << ", Right: " << right << endl;//DEBUGGING
  //cout << "velOut ''" << velOut << ", yawOut " << yawOut << endl;//DEBUGGING
  //cout << "errorYaw: " << errorYaw << ", setPointYaw: " << setPointYaw << endl;//DEBUGGING

  //prevent combine output from going over tihs value
  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}
  this->left = (left/scale);//divides by scale factor, should normally be 1
  this->right = (right/scale);
}

void DriveController::slowPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw)
{
  //cout << "PID SLOW" << endl; //DEBUGGING CODE

  float velOut = slowVelPID.PIDOut(errorVel, setPointVel);
  float yawOut = slowYawPID.PIDOut(errorYaw, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}

void DriveController::constPID(float erroVel,float constAngularError, float setPointVel, float setPointYaw)
{

  //cout << "PID CONST" << endl; //DEBUGGING CODE

  float velOut = constVelPID.PIDOut(erroVel, setPointVel);
  float yawOut = constYawPID.PIDOut(constAngularError, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}


void DriveController::SetVelocityData(float linearVelocity,float angularVelocity)
{
  this->linearVelocity = linearVelocity;
  this->angularVelocity = angularVelocity;
}




PIDConfig DriveController::fastVelConfig()
{
  PIDConfig config;

  config.Kp = 60; //proportional constant
  config.Ki = 10; //integral constant
  config.Kd = 2; //derivative constant
  config.satUpper = 255; //upper limit for PID output
  config.satLower = -255; //lower limit for PID output
  config.antiWindup = config.satUpper; //prevent integral from acruing error untill proportional output drops below a certain limit
  config.errorHistLength = 4; //how many time steps to average error over
  config.alwaysIntegral = true; //should the integral alway be on or only when there is error
  config.resetOnSetpoint = true; //reset the integral and error history whent he setpoint changes
  config.feedForwardMultiplier = 610; //gives 127 pwm at 0.4 commandedspeed  ORIG:320
  config.integralDeadZone = 0.01; //set the integral dead zone, prevented integral from growing or shrinking do to noise
  config.integralErrorHistoryLength = 10000; //how many time ticks should error history should be stored for integration
  config.integralMax = config.satUpper/2; //what is the limit of the integral output for the PID
  config.derivativeAlpha = 0.7; //dead code not used

  return config;

}

PIDConfig DriveController::fastYawConfig() {
  PIDConfig config;

  config.Kp = 60;
  config.Ki = 15;
  config.Kd = 5;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/6;
  config.errorHistLength = 4;
  config.alwaysIntegral = false;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/3;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::slowVelConfig() {
  PIDConfig config;

  config.Kp = 100;
  config.Ki = 8;
  config.Kd = 1.1;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/2;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::slowYawConfig() {
  PIDConfig config;

  config.Kp = 70;
  config.Ki = 16;
  config.Kd = 10;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/4;
  config.errorHistLength = 4;
  config.alwaysIntegral = false;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/6;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::constVelConfig() {
  PIDConfig config;

  config.Kp = 60;
  config.Ki = 10;
  config.Kd = 2;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 610; //gives 127 pwm at 0.4 commandedspeed  ORIG:320
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::constYawConfig() {
  PIDConfig config;

  config.Kp = 5;
  config.Ki = 5;
  config.Kd = 0;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/4;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper;
  config.derivativeAlpha = 0.6;

  return config;

}
