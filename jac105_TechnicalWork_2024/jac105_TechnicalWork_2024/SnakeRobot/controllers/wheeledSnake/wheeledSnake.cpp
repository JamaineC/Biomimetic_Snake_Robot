// File:          wheeledSnake.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#include <array>
#include <string>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

void serpentineWheelMovement(double speed, Motor *wheelMotor[], double time){
  for (int i = 0; i<16; i++){
    if (i<4){
      if (i%2 == 0) wheelMotor[i]->setVelocity(speed);
      else wheelMotor[i]->setVelocity(-speed);
  	
    }
    else if (i < 8){
      if ( i%2 == 0) wheelMotor[i]->setVelocity(-speed);
      else wheelMotor[i]->setVelocity(speed);
    }
    else if (i < 12){
      if (i%2 == 0) wheelMotor[i]->setVelocity(speed);
      else wheelMotor[i]->setVelocity(-speed);
    }
    else{
      if (i%2 == 0) wheelMotor[i]->setVelocity(-speed);
      else wheelMotor[i]->setVelocity(-speed);
    }
  }
}

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

  // get the time step of the current world.
  int timeStep = (int)snakebot->getBasicTimeStep();
  
  int turnCounter = 50; // 90 DEGREE TURN

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
     // set up joint motors
   Motor *wheelMotor[16];
   
   std::string wheelNames[16] = {
   "wheel1","wheel2","wheel3","wheel4",
   "wheel5","wheel6","wheel7","wheel8",
   "wheel9","wheel10","wheel11","wheel2",
   "wheel13","wheel14","wheel15","wheel16"
   };
   

  for (int i = 0; i < 16; i++) {
  // Get the motor the wheels
  wheelMotor[i] = snakebot->getMotor(wheelNames[i]);       
  wheelMotor[i]->setVelocity(0.0); // Start with no motion
  wheelMotor[i]->setPosition(INFINITY); // continuous movement
  }
  
     // set up joint motors
   Motor *lateralJointMotors[3];
   Motor *verticleJointMotors[3];
   std::string jointNames[6] = {
     "link1","link2","link3",
     "link12","link22","link32"
     };

  for (int i = 0; i < 6; i++) {
  // Get the motor for hinge2's first axis
  
  if (i<4){
    lateralJointMotors[i] = snakebot->getMotor(jointNames[i]);       
    lateralJointMotors[i]->setVelocity(0.0); // Start with no verticle movement
    lateralJointMotors[i]->setPosition(INFINITY); // continuous movement
  }
  // Get the motor for hinge2's second axis
  else{
  verticleJointMotors[i-4] = snakebot->getMotor(jointNames[i]);       
  verticleJointMotors[i-4]->setVelocity(0.0); // Start with lateral movement
  verticleJointMotors[i-4]->setPosition(INFINITY); // reset snake
    }
  }

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (snakebot->step(timeStep) != -1) {

    
    if (turnCounter > 25) { // 45 degrees on way
       double speed = 10.0;
       turnCounter--;
       serpentineWheelMovement(speed, wheelMotor,timeStep);

    }
    else if (turnCounter > 0) { //45 degrees the other way
       double speed = -10.0; 
       turnCounter--;
       serpentineWheelMovement(speed, wheelMotor, timeStep);
    }
    else turnCounter = 50;

  };

  // Enter here exit cleanup code.

  delete snakebot;
  return 0;
}
