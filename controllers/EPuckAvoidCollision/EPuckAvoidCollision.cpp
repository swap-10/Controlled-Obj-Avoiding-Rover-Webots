// File:          EPuckAvoidCollision.cpp
// Date:          14-06-2021
// Description:   EPuck controller code to avoid collisions
// Author:        Swapnal Varma
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include<webots/Robot.hpp>
#include<webots/DistanceSensor.hpp>
#include<webots/Motor.hpp>
#include<webots/keyboard.hpp>

#define TIME_STEP 64
// All the webots classes are defined in the "webots" namespace
using namespace webots;

void check_keyboard(Keyboard* k, double* ls, double* rs, int* md)
{
  //Keyboard* k=r->getKeyboard();
  int key=k->getKey();
  //std::cout<<"KKEEEYYYYYY ISSSSSS::::::::::::"<<key<<"\n";
  switch(key)
  {
    case Keyboard::UP:
    printf("Up button pressed\n");
    *ls<*rs?*rs=*ls:*ls=*rs;
    *ls+=0.03;
    *rs+=0.03;
    
    break;
    case Keyboard::DOWN:
    std::cout<<"Down button pressed\n";
    *ls<*rs?*rs=*ls:*ls=*rs;
    *ls-=0.03;
    *rs-=0.03;
    break;
    
    case Keyboard::LEFT:
    std::cout<<"Left button pressed\n";
    //*ls-=0.03;
    *rs+=0.03;
    break;
    
    case Keyboard::RIGHT:
    std::cout<<"Right button pressed\n";
    *ls+=0.03;
    //*rs-=0.03;
    break;
    
    case int('A'):
    *md=0;
    printf("Autodrive ON\n");
    break;
  }
  printf("%.2f %.2f\n", *ls, *rs);
}


void check_keyb(Keyboard* k, int* md)
{
  //Keyboard* k=r->getKeyboard();
  int key=k->getKey();
  //std::cout<<"KKEEEYYYYYY ISSSSSS::::::::::::"<<key<<"\n";
  switch(key)
  { 
    case int('A'):
    *md=0;
    printf("Autodrive ON\n");
    break;
    
    case int('M'):
    *md=1;
    printf("Manual drive ON\n");
    break;
  }
}

// This is the main program of the controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  
  DistanceSensor *ps[8];
  char psNames[8][4]={"ps0", "ps1", "ps2", "ps3", "ps4",
                      "ps5", "ps6", "ps7"
                      };
  for(int i=0; i<8; i++)
  {
    ps[i]=robot->getDistanceSensor(psNames[i]);
    ps[i]->enable(TIME_STEP);
  }
  
  Motor *leftMotor=robot->getMotor("left wheel motor");
  Motor *rightMotor=robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  Keyboard* k=robot->getKeyboard();
  k->enable(TIME_STEP);
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
    #define ms 6.28
    double left_speed=(0*ms);
    double right_speed=(0*ms);
    int md=1;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) 
  {
     if(md)
     {
      check_keyboard(k, &left_speed, &right_speed, &md);
     }
     else
     {
       check_keyb(k, &md);
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    double psValues[8];
    for(int i=0; i<8; i++)
    {
      psValues[i]=ps[i]->getValue();
    }

    // Process sensor data here.
    
    bool right_obstacle= psValues[0]>80.0||psValues[1]>80.0||
                         psValues[2]>80.0;
    bool left_obstacle=psValues[5]>80.0||psValues[6]>80.0||
                       psValues[7]>80.0;
    
    
    if(left_obstacle&&right_obstacle)
    {
      left_speed=(0.5*ms);
      right_speed=(-0.5*ms);
    }
    
    else if(left_obstacle)
    {
      left_speed=(0.5*ms);
      right_speed=(-0.5*ms);
    }
    
    else if(right_obstacle)
    {
      left_speed=(-0.5*ms);
      right_speed=(0.5*ms);
    }
    else
    {
      left_speed=0.5*ms;
      right_speed=0.5*ms;
    }
    }
    
    leftMotor->setVelocity(left_speed);
    rightMotor->setVelocity(right_speed);
    
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  }

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
