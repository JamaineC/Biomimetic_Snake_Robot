// File:          first_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Lidar.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Compass.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Camera.hpp>
#include <cmath>



#include <iostream> 
#include <array>
#include <string>

#define THRESHOLD 0.5
//#define STRAIGHTEN

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

const double amplitudeLateral = 0.5; // Max angle for joint oscillation in radians
const double amplitudeVerticle = 0.175; // Max angle for joint oscillation in radians
const double frequency = 1.0; // Oscillations per second

// PID coefficients
double yawKp = 0.1; // Proportional gain
double yawKi = 0.01; // Integral gain    
double yawKd = 0.00; // Derivative gain
double pitchKp = 0.0; // Proportional gain
double pitchKi = 0.0; // Integral gain    
double pitchKd = 0.0; // Derivative gain

// PID error variables
double yawPreviousError = 0.0;
double pitchPreviousError = 0.0;
double yawIntegralError = 0.0;
double pitchIntegralError = 0.0;


double yawPIDOutput; // initialise as zero - no longer in use
double pitchPIDOutput; // initialise as zero - no longer in use

double initialDirection; // used for staying straight

bool isForwardsSet = false;  // used for moving forwards - no longer in use
double desiredYaw; // used to keep robot straight - no longer in use
#define YAW_THRESHOLD 2 * M_PI * 5/360

const double objectDetectionThreshold = 0.9; // RangeFinder threshold for object detection
#define WALL_THRESHOLD 1.0
#define SENSOR_THRESHOLD 0.5
#define TURN_THRESHOLD 0.5    
// 0.15 for sidewinder
int sensorUpdateCounter = 0;
int updateInterval = 10; // wait 20 timeSteps to do an update

/* variables for rangeFinder readings*/
float averageDistanceFront;
float averageDistanceLeft;
float averageDistanceRight;
float averageDistanceBack; 

float leftDS;
float rightDS;

                            /*   SENSOR FUNCTIONS   */
                            
                            
void adjustVerticalPhaseOffsets(double verticlePhaseOffsets[], TouchSensor *forceSensors[], int numJoints) {
    double forceReadings[numJoints + 1];  // One more sensor than joints
    double totalForce = 0; // total force acting on all segemnts 
    for (int i = 0; i <= numJoints + 1; i++) {  // Iterate over all sensors
        forceReadings[i] = forceSensors[i]->getValue();
        if (forceReadings[i] > 0 ){ // if force applied on segment
        totalForce += forceReadings[i]; // add the force
          
        }
    }

    if (totalForce == 0) return; // to avoid division by zero error

    for (int i = 0; i < numJoints; i++) {
        // If last motor average last two force sensors else take the force value
        double currentForce = (i == numJoints - 1) ? (forceReadings[i] + forceReadings[i + 1]) / 2 : forceReadings[i];
        
        double ratioOfForce = currentForce / totalForce;  // proportionate force
        
        verticlePhaseOffsets[i] += (M_PI/2.0) * (1.0 - ratioOfForce); // adjust offset based on ratio of force on segment | circular movement so repeats
    }
}

// Function to get the average distance of a rangeFinder
float getAverageDistance(const RangeFinder *rangeFinder) {
  const float *rangeImage = rangeFinder->getRangeImage(); // get range image of the range finder
  int width = rangeFinder->getWidth(); // get width of range image
  int height = rangeFinder->getHeight(); // get height of range image
  int size = width * height; // calculate resolution of range image
  
  float sum = 0.0; // float for combined distance of the different ranges
  int count = 0; // index counter for number of ranges added
  
  for (int i = 0; i < size; i++) {
    float distance = rangeImage[i];
    // if distance is within a valid range.
    if (distance >= (rangeFinder->getMinRange())  && distance <= rangeFinder->getMaxRange()) { 
      sum += distance; // add to total distance
      count++; // increase the count index
    }
  }

   float average = sum / count; // average distance for that rangeFinnder
  
  
  // check if count above zero and then return avarage
  return count > 0 ? average : rangeFinder->getMaxRange();
}

void detectColourRed(Camera *camera[]){

  int numberOfObjectsFront = camera[0]->getRecognitionNumberOfObjects(); // get number of detected objects from front camera
  const CameraRecognitionObject *objectsFront = camera[0]->getRecognitionObjects(); // set all detected objects
  int numberOfObjectsLeft = camera[1]->getRecognitionNumberOfObjects(); // get number of detected objects from left camera
  const CameraRecognitionObject *objectsLeft = camera[1]->getRecognitionObjects(); // set all detected objects
  int numberOfObjectsRight = camera[2]->getRecognitionNumberOfObjects(); // get number of detected objects from right camera
  const CameraRecognitionObject *objectsRight = camera[2]->getRecognitionObjects(); // set all detected objects
  int numberOfObjectsBack = camera[3]->getRecognitionNumberOfObjects(); // get number of detected objects from back camera
  const CameraRecognitionObject *objectsBack = camera[3]->getRecognitionObjects(); // set all detected objects
  
  for (int i = 0; i < numberOfObjectsFront; ++i) {
    int red = objectsFront[i].colors[0]; // count the number of reds
    int green = objectsFront[i].colors[1]; // count the number of greens
    int blue = objectsFront[i].colors[2]; // count the number of blue
  
    // //Check if the object's color is  red
    if (red > THRESHOLD && green < THRESHOLD && blue < THRESHOLD) {
      cout << "red object found" << endl;
      break;
    }
  }
  
}

                            /* ********************* */
                            
        

                            /* LOCOMOTIVE FUNCTIONS */
                            
                            
// Function to start the robot's motion
void startRobot(Motor *lateralJointMotors[], Motor *verticleJointMotors[], int numJoints){
    for (int i = 0; i < numJoints; i++) {
      lateralJointMotors[i]->setVelocity(10.0); // set lateral joint motors speed
      verticleJointMotors[i]->setVelocity(10.0); // set verticle joint motors speed
      //cout << "snakebot has started moving" << endl;
    }
}

// Function to stop the robot's motion
void stopRobot(Motor *lateralJointMotors[], Motor *verticleJointMotors[], int numJoints){
    for (int i = 0; i < numJoints; i++) {
      lateralJointMotors[i]->setVelocity(0.0); // stop the movement of the lateral joints
      verticleJointMotors[i]->setVelocity(0.0); // stop the movement of the verticle joint
      cout << "snakebot has stopped moving" << endl;
    }
}

void setJointAngles(Motor *lateralJointMotors[], double lateralPhaseOffsets[], Motor *verticleJointMotors[], double verticlePhaseOffsets[], double time, int numJoints){
     // //error in the angle of the neck of the snakebot
  // double pitchError = desiredPitch - pitch;
  
  // if(pitchError > M_PI) pitchError -= 2.0 * M_PI;
  // if(pitchError < -M_PI) pitchError += 2.0 * M_PI;
    
   // //Integral of error (sum of Error)
   // pitchIntegralError += pitchError * (timeStep)/1000; // need ms->s conversion
  
   // //Derivative of error (difference in error)
  // double pitchDerivativeError = (pitchError - pitchPreviousError) / (timeStep / 1000.0); // need ms->s conversion
  
   // //PID output config
  // double pitchPIDOutput = pitchKp * pitchError + pitchKi * pitchIntegralError + pitchKd * pitchDerivativeError;
  
   // //Save current error as previous error for next iteration
  // pitchPreviousError = pitchError;
  
  for (int i = 0; i < numJoints; i++) {
    double jointAngleLateral = amplitudeLateral * sin(2.0 * M_PI * frequency * time + lateralPhaseOffsets[i]); // displacement of a point in simple hamronic motion
    double jointAngleVerticle = amplitudeVerticle * sin(2.0 * M_PI * frequency * time + verticlePhaseOffsets[i]); // displacement of a point in simple harmonic motion

    lateralJointMotors[i]->setPosition(jointAngleLateral); // set position of joint angle
    verticleJointMotors[i]->setPosition(jointAngleVerticle);  // set position of joint angle
    } 
}



// Function to set joint angles for forward sinsusodial movement
void moveForward(Motor *lateralJointMotors[], Motor *verticleJointMotors[], double time, int numJoints,InertialUnit *imu,TouchSensor *forceSensors[]) {
  double lateralPhaseOffsets[] = {M_PI/2.0, M_PI/2.0 , M_PI , M_PI}; // phase-shifts for the lateral joint angles
  double verticlePhaseOffsets[] = {M_PI/2.0, M_PI/2.0, M_PI/2.0, M_PI/2.0}; // phase-shifts for the verticle joint angles
   setJointAngles(lateralJointMotors, lateralPhaseOffsets, verticleJointMotors, verticlePhaseOffsets, time, numJoints);
        
      /* commented out yaw adjustment logic */
      
      
          // float roll = imu->getRollPitchYaw()[0]; // get the current roll
          // float pitch = imu->getRollPitchYaw()[1]; // get the current pitch
          // float yaw = imu->getRollPitchYaw()[2]; // get the currentYaw

    // If yaw error exceeds the threshold, call turning functions to correct it
    // if (yawError > YAW_THRESHOLD) {
        // cout << "turning right, to straighten" << endl;
        // turnRight(lateralJointMotors, verticleJointMotors, time, numJoints);
    // } else if (yawError < -YAW_THRESHOLD) {
        // cout << "turning left, to straighten" << endl;
        // turnLeft(lateralJointMotors, verticleJointMotors, time, numJoints);
    // } else {
        // Normal forward movement using sinusoidal joint angles     
               
                /* commented out PID controllers */

   //error in the angle of the neck of the snakebot
   // double yawError = desiredYaw - yaw;
 
   // //stop snake wrapping around, ensure shortest turning angle
  // if(yawError > M_PI) yawError -= 2.0 * M_PI;
  // if(yawError < -M_PI) yawError += 2.0 * M_PI;
  
  // //Integral of error (sum of Error)
   // yawIntegralError += yawError * (time)/1000; // need ms->s conversion

  // //Derivative of error (difference in error)
  // double yawDerivativeError = (yawError - yawPreviousError) / (time / 1000.0); // need ms->s conversion

  // //PID output config
  // double yawPIDOutput = yawKp * yawError + yawKi * yawIntegralError + yawKd * yawDerivativeError;

  // //Save current error as previous error for next iteration
  // yawPreviousError = yawError;

// cout << "Yaw PID Output: " << yawPIDOutput << " | Pitch PID Output: " << pitchPIDOutput << endl;
  // adjustVerticalPhaseOffsets(verticlePhaseOffsets, forceSensors, numJoints); // adjust verticle offset based on previous force 

}


// Function to turn left
void turnLeft(Motor *lateralJointMotors[], Motor *verticleJointMotors[], double time, int numJoints){
  double lateralPhaseOffsets[] = {M_PI/2.0, M_PI , M_PI/2.0 , M_PI}; // phase-shifts for the lateral joint angles
  double verticlePhaseOffsets[] = {M_PI/2.0, M_PI, M_PI/2.0, M_PI}; // phase-shifts for the verticle joint angles
  setJointAngles(lateralJointMotors, lateralPhaseOffsets, verticleJointMotors, verticlePhaseOffsets, time, numJoints);
    // cout << "snakebot turning left" << endl;
}

// Function to turn right
void turnRight(Motor *lateralJointMotors[], Motor *verticleJointMotors[], double time, int numJoints){
  double lateralPhaseOffsets[] = {M_PI, M_PI/4.0 , M_PI , M_PI/4.0}; // phase-shifts for the lateral joint angles
  double verticlePhaseOffsets[] = {M_PI/2.0, M_PI/2.0, M_PI/2.0 , M_PI/2.0}; // phase-shifts for the verticle joint angles
  setJointAngles(lateralJointMotors, lateralPhaseOffsets, verticleJointMotors, verticlePhaseOffsets, time, numJoints);
  // cout << "snakebot turning right" << endl;
}

// Function to move the snake to the left laterally
void moveLeft(Motor *lateralJointMotors[], Motor *verticleJointMotors[],double time, int numJoints){
  double lateralPhaseOffsets[] = {M_PI/2.0, M_PI/2.0 , M_PI , M_PI}; // phase-shifts for the lateral joint angles
  double verticlePhaseOffsets[] = {M_PI/2.0, M_PI/2.0, M_PI/2.0, M_PI/2.0}; // phase-shifts for the verticle joint angles
  setJointAngles(lateralJointMotors, lateralPhaseOffsets, verticleJointMotors, verticlePhaseOffsets, time, numJoints);
    // cout << "snakebot moving left" << endl;
}

// Function to move the snake to the right laterally
void moveRight(Motor *lateralJointMotors[], Motor *verticleJointMotors[], double time, int numJoints){
  double lateralPhaseOffsets[] = {M_PI/2.0, M_PI/2.0 , M_PI/2.0, M_PI/2.0}; // phase-shifts for the lateral joint angles
  double verticlePhaseOffsets[] = {M_PI/4.0, M_PI, M_PI/4.0 , M_PI}; // phase-shifts for the verticle joint angles
  setJointAngles(lateralJointMotors, lateralPhaseOffsets, verticleJointMotors, verticlePhaseOffsets, time, numJoints);
    // cout << "snakebot moving right" << endl;
}

                          /******************************/





                           /* OBSTACLE AVOIDANCE FUNCTIONS */


// function for basic object avoidance using serpentine snake movement
void serpentineAvoidance(RangeFinder *rangeFinder[], DistanceSensor *distanceSensor[], Motor *lateralJointMotors[], Motor *verticleJointMotors[], double time, int numJoints, InertialUnit *imu, TouchSensor *forceSensor[]){

  RangeFinder *frontRangeFinder = rangeFinder[0];
  RangeFinder *leftRangeFinder = rangeFinder[1];
  RangeFinder *rightRangeFinder = rangeFinder[2];
  RangeFinder *backRangeFinder = rangeFinder[3];

   
   // delay sensor update so the robot can move smoothly
   if (sensorUpdateCounter++ % updateInterval == 0) { // update every x timeSteps
    averageDistanceFront = getAverageDistance(frontRangeFinder);
    averageDistanceLeft = getAverageDistance(leftRangeFinder);
    averageDistanceRight = getAverageDistance(rightRangeFinder);
    averageDistanceBack = getAverageDistance(backRangeFinder);
    leftDS = distanceSensor[0]->getValue();
    rightDS = distanceSensor[1]->getValue();
    // Reset counter
    sensorUpdateCounter = 0;
    
  }

  if (averageDistanceFront < objectDetectionThreshold && averageDistanceFront > 0.0) {
    // if object in front detected then turn away from it 
    isForwardsSet = false;
    turnLeft(lateralJointMotors, verticleJointMotors, time, numJoints); // turn left
    cout << "turning away from object in front" << endl;
  }
  else if (leftDS < objectDetectionThreshold) { // if object to front-left 
      isForwardsSet = false;
      turnRight(lateralJointMotors, verticleJointMotors, time, numJoints);
      cout << "turning away from object in front-left" << endl;
  }
  else if (rightDS < objectDetectionThreshold) { // if object to front-right , turn left
      turnLeft(lateralJointMotors, verticleJointMotors, time, numJoints); // turn left
    
  }
  // if object to the left then move right 
  else if(((averageDistanceLeft < objectDetectionThreshold) && (averageDistanceLeft != 0.0)) || ((distanceSensor[0]->getValue() < objectDetectionThreshold + 0.25) && (distanceSensor[0]->getValue() > 0)) || ((distanceSensor[2]->getValue() < objectDetectionThreshold) && (distanceSensor[2]->getValue() > 0))) {
      moveRight(lateralJointMotors, verticleJointMotors, time, 4);
  }
  // if object in (right-side)
  else if (((averageDistanceRight < objectDetectionThreshold) && (averageDistanceRight != 0.0)) || ((distanceSensor[1]->getValue() < objectDetectionThreshold + 0.25) && (distanceSensor[1]->getValue() > 0)) || (distanceSensor[3]->getValue() < objectDetectionThreshold && distanceSensor[3]->getValue() > 0)) {
     moveLeft(lateralJointMotors, verticleJointMotors, time, numJoints);
  }
  else if ((averageDistanceBack < objectDetectionThreshold) && (averageDistanceBack > 0)) {
   moveLeft(lateralJointMotors, verticleJointMotors, time, numJoints);
  }
  else {
      // continue moving with sideways motion
      startRobot(lateralJointMotors, verticleJointMotors, numJoints);
      moveForward(lateralJointMotors, verticleJointMotors, time, numJoints,imu,forceSensor);
      cout << "moving with serpentine movment " << endl;
  }
  
}

// function for basic object avoidance using sidewinder snake movement
void sideWinderAvoidance(RangeFinder *rangeFinder[], DistanceSensor *distanceSensor[], Motor *lateralJointMotors[], Motor *verticleJointMotors[], double time, int numJoints){

  RangeFinder *frontRangeFinder = rangeFinder[0];
  RangeFinder *leftRangeFinder = rangeFinder[1];
  RangeFinder *rightRangeFinder = rangeFinder[2];
  RangeFinder *backRangeFinder = rangeFinder[3];
  
  
  // delay sensor update, so snake can move faster (sensors do not need to run every timeStep)
   
   
   if (sensorUpdateCounter++ % updateInterval == 0) { // update every x timeSteps
    averageDistanceFront = getAverageDistance(frontRangeFinder);
    averageDistanceLeft = getAverageDistance(leftRangeFinder);
    averageDistanceRight = getAverageDistance(rightRangeFinder);
    averageDistanceBack = getAverageDistance(backRangeFinder);
    // Reset counter
    sensorUpdateCounter = 0;
  }
   
  
  if (distanceSensor[2]->getValue() < objectDetectionThreshold && distanceSensor[3]->getValue() < objectDetectionThreshold) { // if bottom of snake stuck in corner
    turnLeft(lateralJointMotors, verticleJointMotors, time, numJoints); // turn left
  }
  else if (distanceSensor[0]->getValue() < objectDetectionThreshold && distanceSensor[1]->getValue() < objectDetectionThreshold) { // if top of snake stuck in corner
    turnRight(lateralJointMotors, verticleJointMotors, time, numJoints); // turn right
  }
  else if (distanceSensor[1]->getValue() < objectDetectionThreshold ) { // if object too close to left
    turnRight(lateralJointMotors, verticleJointMotors, time, numJoints); // turn right
    cout << "adjusting to the right" << endl;
  }
  else if (distanceSensor[3]->getValue() < objectDetectionThreshold) { // if object too to the right
    turnLeft(lateralJointMotors, verticleJointMotors, time, numJoints); // turn left
    cout << "adjusting to the Left" << endl;
  } 
  else if (averageDistanceRight < objectDetectionThreshold && averageDistanceRight > 0){
    moveLeft(lateralJointMotors, verticleJointMotors, time, numJoints); // move back if needed
    cout << "object ahead, turning left" << endl;
  }  
  else if (averageDistanceBack < objectDetectionThreshold && averageDistanceBack > 0){
  turnLeft(lateralJointMotors, verticleJointMotors, time, numJoints); // move back if needed
  cout << "object back-left, moving left" << endl;
  }     
  else {
      // continue moving with sideways motion
      startRobot(lateralJointMotors, verticleJointMotors, numJoints); // start robots movement
      moveRight(lateralJointMotors, verticleJointMotors, time, numJoints); // move to the right (sidewinding)
      cout << "moving with sidewinder movment " << endl;
  }
  

}
                          /******************************/

                           /* MAZE TRAVERSAL FUNCTIONS */
                           
// function for maze traversal using serpentine snake movement
void serpentineMazeControl(RangeFinder *rangeFinder[], DistanceSensor *distanceSensor[], Motor *lateralJointMotors[], Motor *verticleJointMotors[], double time, int numJoints,InertialUnit *imu, TouchSensor *forceSensor[]){

  // set up rangeFinders
  RangeFinder *frontRangeFinder = rangeFinder[0];
  RangeFinder *leftRangeFinder = rangeFinder[1];
  RangeFinder *rightRangeFinder = rangeFinder[2];
  RangeFinder *backRangeFinder = rangeFinder[3];
  
   // delay sensor update, so snake can move faster (sensors do not need to run every timeStep)
   if (sensorUpdateCounter++ % updateInterval == 0) { // update every x timeSteps
    averageDistanceFront = getAverageDistance(frontRangeFinder);
    averageDistanceLeft = getAverageDistance(leftRangeFinder);
    averageDistanceRight = getAverageDistance(rightRangeFinder);
    averageDistanceBack = getAverageDistance(backRangeFinder);
    // Reset counter
    sensorUpdateCounter = 0;
}

   float distanceDifference = averageDistanceLeft - averageDistanceRight;
   
       // Determine the direction to turn based on the distance difference.
    if (distanceDifference < -TURN_THRESHOLD) {
        // If the difference is positive and above the threshold, turn right.
        turnRight(lateralJointMotors, verticleJointMotors, time, numJoints);
        cout << "Distance difference " << distanceDifference << ", turning right." << endl;
    } 
    else if (distanceDifference > TURN_THRESHOLD) {
        // If the difference is negative and below the negative threshold, turn left.
        turnLeft(lateralJointMotors, verticleJointMotors, time, numJoints);
        cout << "Distance difference " << distanceDifference << ", turning left." << endl;
    }
         // //left turn available
    else if ((averageDistanceFront < WALL_THRESHOLD && averageDistanceFront > 0) && (averageDistanceRight < WALL_THRESHOLD && averageDistanceRight > 0)){
        turnLeft(lateralJointMotors, verticleJointMotors, time, numJoints);
        cout << "left turn available, turning left" << endl;
  
    }
      // //right turn available
    else if ((averageDistanceFront < WALL_THRESHOLD && averageDistanceFront > 0) && (averageDistanceLeft < WALL_THRESHOLD && averageDistanceLeft > 0)){
        turnRight(lateralJointMotors, verticleJointMotors, time, numJoints);
        cout << "Right turn available, turning right" << endl;
    }

    else {
        // If the difference is within the threshold, keep moving sidewinder to the right.
        moveForward(lateralJointMotors, verticleJointMotors, time, numJoints,imu,forceSensor);
        cout << "Distance difference " << distanceDifference << ", moving forward." << endl;
    }

}


// function for maze traversal using sidewinder snake movement
void sideWinderMazeControl(RangeFinder *rangeFinder[], DistanceSensor *distanceSensor[], Motor *lateralJointMotors[], Motor *verticleJointMotors[], double time, int numJoints){
 //right moving sidwinder snake

  RangeFinder *frontRangeFinder = rangeFinder[0]; // set front range finder
  RangeFinder *leftRangeFinder = rangeFinder[1]; // set left range finder
  RangeFinder *rightRangeFinder = rangeFinder[2]; // set right range finder
  RangeFinder *backRangeFinder = rangeFinder[3]; // set back range finder
  
   
  averageDistanceFront = getAverageDistance(frontRangeFinder); // average distance for front rangeFinder
  averageDistanceLeft = getAverageDistance(leftRangeFinder); // average distance for left rangeFinder
  averageDistanceRight = getAverageDistance(rightRangeFinder); // average distance for right rangeFinder
  averageDistanceBack = getAverageDistance(backRangeFinder); // average distance for back rangeFinder
   
   
   float distanceDifference = averageDistanceFront - averageDistanceBack; // used for turning 
   
    if (distanceDifference < -TURN_THRESHOLD) { // if too far on the left
        // If the difference is negative and below the threshold, turn right.
        turnRight(lateralJointMotors, verticleJointMotors, time, numJoints);
        cout << "Distance difference " << distanceDifference << ", turning right." << endl;
    } 
    else if (distanceDifference > TURN_THRESHOLD) { // 
        // If the difference is positive and above the negative threshold, turn left.
        turnLeft(lateralJointMotors, verticleJointMotors, time, numJoints);
        cout << "Distance difference " << distanceDifference << ", turning left." << endl;
    }
         // //left turn available
    else if ((averageDistanceRight < WALL_THRESHOLD && averageDistanceRight > 0) && (averageDistanceBack < WALL_THRESHOLD && averageDistanceBack > 0)){
        turnLeft(lateralJointMotors, verticleJointMotors, time, numJoints);
        cout << "left turn available, turning left" << endl;
  
    }
      // //right turn available
    else if ((averageDistanceRight < WALL_THRESHOLD && averageDistanceRight > 0) && (averageDistanceFront < WALL_THRESHOLD && averageDistanceFront > 0)){
        turnRight(lateralJointMotors, verticleJointMotors, time, numJoints);
        cout << "Right turn available, turning right" << endl;
    }
    

    else {
        //  keep moving sidewinder to the right through the maze.
        moveRight(lateralJointMotors, verticleJointMotors, time, numJoints);
        cout << "Distance difference " << distanceDifference << ", moving forward." << endl;
    }  
}

                          /******************************/

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {


  // create the Robot instance.
  Robot *snakebot = new Robot();
  int timeStep = (int)snakebot->getBasicTimeStep(); // get the time step of the current world.
  int sensorDelay = 2.0; // amount of timeSteps to delay for sensors so movement can occur smoothly


  // set up distance sensors
  DistanceSensor *distanceSensor[4];
  std::string dsNames[4] = {"frontLeftDs", "frontRightDs","backLeftDs","backRightDs"};
  for (int i = 0; i < 4; i++) {
    distanceSensor[i] = snakebot->getDistanceSensor(dsNames[i]);
    distanceSensor[i]->enable(timeStep * sensorDelay);
  }
   
   
   // set up joint motors
   Motor *lateralJointMotors[4];
   Motor *verticleJointMotors[4];
   std::string jointNames[8] = {"link1","link2","link3","link4","link12","link22","link32","link42"};

  for (int i = 0; i < 8; i++) {
  // Get the motor for hinge2's first axis
  
  if (i<4){
    lateralJointMotors[i] = snakebot->getMotor(jointNames[i]);       
    lateralJointMotors[i]->setVelocity(10.0); // Start with no verticle movement
    lateralJointMotors[i]->setPosition(0.0); // reset snake
  }
  // Get the motor for hinge2's second axis
  else{
  verticleJointMotors[i-4] = snakebot->getMotor(jointNames[i]);       
  verticleJointMotors[i-4]->setVelocity(10.0); // Start with lateral movement
  verticleJointMotors[i-4]->setPosition(0.0); // reset snake
    }
  }

  // set up touch sensors
  TouchSensor *forceSensors[5];
  std::string forceSensorNames[5] = {"neckTouchSensor", "segment1TouchSensor", "segment2TouchSensor", "segment3TouchSensor", "segment4TouchSensor"};

  //setup force sensors for each segment
  for (int i = 0; i < 5; i++) {
    forceSensors[i] = snakebot->getTouchSensor(forceSensorNames[i]);
    forceSensors[i]->enable(timeStep * sensorDelay);
  }
  
  // Setup the RangeFinder
  RangeFinder *rangeFinder[4];
  std::string rangeFinderNames[4] = {"rangeFinder","leftRangeFinder","rightRangeFinder","backRangeFinder"};
  for (int i = 0; i< 4; i++){
  rangeFinder[i] = snakebot->getRangeFinder(rangeFinderNames[i]);  
      if (rangeFinder[i] == nullptr) {
      cerr << "No RangeFinder was found with the selected name." << endl;
      return 1; // Exit if not found to avoid nullptr access later
      }
      rangeFinder[i]->enable(timeStep * sensorDelay);
    }

  //setup gps
  GPS *gps = snakebot->getGPS("gps");
  gps->enable(timeStep); // get gps callback every timestep
  
  //setup imu
  InertialUnit *imu = snakebot->getInertialUnit("imu");
  imu->enable(timeStep); // get imu reading every timestep
  
  // setup camera
  
  Camera *camera[4];
  std:string cameraNames[4] = {"frontCamera","leftCamera","rightCamera","rearCamera"}; // array for camera names
  for (int i = 0; i < 4; i++){
  camera[i] = snakebot->getCamera(cameraNames[i]); // get camera
  camera[i]->enable(timeStep); // set callback for every timestep
  camera[i]->recognitionEnable(timeStep); // enable recognition for objects (could not get to work)
  }

startRobot(lateralJointMotors, verticleJointMotors, 4);

  // Main loop: 
  // - perform simulation steps until Webots is stopping the controller
  while (snakebot->step(timeStep) != -1) {

  double time = snakebot->getTime(); // get current time
  
  detectColourRed(camera);
  
  /* CONTROL LOGIC */
  
  /* comment out the movement types you do not need */
  
  //moveForward(lateralJointMotors, verticleJointMotors, time, 4,imu,forceSensors);
  //moveRight(lateralJointMotors, verticleJointMotors, time, 4);
  //moveLeft(lateralJointMotors, verticleJointMotors, time, 4);
  //turnRight(lateralJointMotors, verticleJointMotors, time, 4);
  //turnLeft(lateralJointMotors, verticleJointMotors, time, 4);
  

  serpentineAvoidance(rangeFinder, distanceSensor, lateralJointMotors, verticleJointMotors, time, 4, imu, forceSensors);
  //sideWinderAvoidance(rangeFinder, distanceSensor, lateralJointMotors, verticleJointMotors, time, 4);

  //serpentineMazeControl(rangeFinder, distanceSensor ,lateralJointMotors, verticleJointMotors, time, 4, imu, forceSensors);
  //sideWinderMazeControl(rangeFinder, distanceSensor, lateralJointMotors, verticleJointMotors, time, 4);

  
  /* ------------- */


    };

// maybe flip joint angle by reversing for loop to reverse snake    did not work
  // Enter here exit cleanup code.
  delete snakebot;
  return 0;
}


