#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < waypointTolerance) {// 15 cm
      attemptCount = 0;
    }
  }

  if (attemptCount > 0 && attemptCount < 5) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= 5 || attemptCount == 0)
  {
    attemptCount = 0;//TODO figure out this

    result.type = waypoint;
    Point  searchLocation;

    //searchLocation.x = currentLocation.x - (0.5 * cos(currentLocation.theta));//should go forward .5 meters
    //searchLocation.y = currentLocation.y - (0.5 * sin(currentLocation.theta));
///////////////////////////////
//cant quite get the angles to work, swarmie can go forwards and backwards well enough with +- .5*sin/cos(currentLocation.theta)
//may want to adjust angle tollerance smaller to make more accurate
//not able to get it to go the correct turn dirrection though. sometimes works but isnt too consistent
//most likely do to just going off imu readings, it should be to just adjust the theta value to determine what dirrection to go
//check yaw value in logs and in DriveController STATE_MACHINE_ROTATE
    /*if(x==1){
      searchLocation.x = currentLocation.x + (0.5 * cos(currentLocation.theta/2));//should go forward .5 meters
      searchLocation.y = currentLocation.y + (0.5 * sin(currentLocation.theta/2));
      x++;
    }else if(x==2){
      //searchLocation.x = currentLocation.x + (0.5 * sin(currentLocation.theta));//should go right .5 meters
      //searchLocation.y = currentLocation.y + (0.5 * cos(currentLocation.theta));
      searchLocation.x = currentLocation.x + (0.5 * cos(currentLocation.theta));//should go forward .5 meters
      searchLocation.y = currentLocation.y + (0.5 * sin(currentLocation.theta));
      x=1;
    }*/

      //select new heading from Gaussian distribution around current heading
      searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));

        //cout << "SearchLoc x = " << searchLocation.x << " curLoc y " << searchLocation.y << endl;//DEBUGGING
        //cout << "curLoc x = " << currentLocation.x << " curLoc y " << currentLocation.y << endl;//DEBUGGING
        /*mess with rotateOnlyangleTolerance in DriveController.h
        center location is being set to ~(0.0,0.0) and locations are releative
        I think every swarmie will have its own relative location so when coding as a swarm we will have to make them compare locations to get the same map, relative for 1 will be different for another I think
        When adjusting the searchlocation x and y it is for the s and y cordinate, x isnt specificly rotate like we though yesterday richard

        the prioirity of everything is set in the bottom functions of logic controller, typically search controller is given lowest priority, so the  values from pickupcontroller/etc could potentially overwrite these
        by using cout in this function it will print in the misc/logs/rover_behavior file so it can be used for DEBUGGING
        someone needs to figure out how to set up QT creator so we can debug differently. although it might just debug the assembly code for tracing I am not sure
        */

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;//only return a single waypoint
  }

}

void SearchController::SetCenterLocation(Point centerLocation) {

  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;

  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }

}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}
