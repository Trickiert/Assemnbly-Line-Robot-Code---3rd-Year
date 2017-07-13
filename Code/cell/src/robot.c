#include <assert.h>
#include <stdint.h>
#include <pwm.h>
#include <interface.h>
#include <robot.h>

#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <buttons.h>
#include <lcd.h>


/* Major cycle of 50Hz (20ms) 
 * Duty cycle ranges from ~1ms to ~2ms
 * i.e. ~5% to ~10% of major cycle 
 * 
 */
enum {PWM_MAJOR_CYCLE_HZ = 50};

/*
 * Constants below represent millionths of the major cycle,
 * e.g. HAND_MIN 45000 means the minimum value of the duty cycle
 * for the hand joint is 45000/1000000 of the major cycle time,
 * i.e. duty cycle time = (20 * (45000/1000000) = 0.9ms
 */
enum {
  //These are the max values the robot can be set to
  //stops things from breaking and our robot from getting arthritis.
  
  DUMMY_MIN = 45000, //Dummy for Testing
  HAND_MIN = 45000,
  WRIST_MIN = 45000,
  ELBOW_MIN = 45000,
  WAIST_MIN = 45000,
  DUMMY_MAX = 100000, 
  HAND_MAX  = 100000,
  WRIST_MAX = 100000,
  ELBOW_MAX = 100000,
  WAIST_MAX = 100000,
  
  //Neutral (Starting) Positions for each joint. will set to these by default on program runtime
  DUMMY_NEUTRAL = 32500,
  HAND_NEUTRAL =  44000,
  WRIST_NEUTRAL = 83750,
  ELBOW_NEUTRAL = 75000,
  WAIST_NEUTRAL = 55000,
};
                           
                          
static const uint32_t jointMinPos[5] = {DUMMY_MIN, HAND_MIN, WRIST_MIN, ELBOW_MIN, WAIST_MIN};
static const uint32_t jointMaxPos[5] = {DUMMY_MAX, HAND_MAX, WRIST_MAX, ELBOW_MAX, WAIST_MAX};
static const uint32_t jointStep = 10; //Inc and Dec speed

//Order of components, needed to so we can move each part independadly using a counter.  (See header)
static uint32_t jointPos[5] = {DUMMY_NEUTRAL, HAND_NEUTRAL, WRIST_NEUTRAL, ELBOW_NEUTRAL, WAIST_NEUTRAL};

void robotInit(void) {
  interfaceInit(ROBOT);
  pwmInit(PWM_MAJOR_CYCLE_HZ);  // set major cycle (should be 20ms period)        
  pwmChannelInit((pwmIdentifier_t)ROBOT_HAND, HAND_NEUTRAL);  
  pwmChannelInit((pwmIdentifier_t)ROBOT_WRIST, WRIST_NEUTRAL); 
  pwmChannelInit((pwmIdentifier_t)ROBOT_ELBOW, ELBOW_NEUTRAL); 
  pwmChannelInit((pwmIdentifier_t)ROBOT_WAIST, WAIST_NEUTRAL); 
}

/*
*************************************************************
*                   ROBOT PATHING                           *
*************************************************************
*/
void robotJointSetState(robotJoint_t joint, robotJointStep_t step) {
  uint32_t newJointPos;
 
  
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  
 step == 1;
  
  //Step 1
  if (step == 1) 
  {
    joint = 1;
   newJointPos = 45750;
    if (newJointPos > jointMaxPos[joint]) {
       newJointPos = jointMaxPos[joint];
       
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  //OSTimeDly(1000);
  
  //Step 1
  if (step == 2)
  {
    joint = 3;
  
  for(float iu = jointPos[joint]; iu > 74250; iu = iu - 0.1)
  {
  newJointPos = iu;
    if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  }
  //OSTimeDly(1000);
  
  //Step 3
  if (step == 3)
  {
  joint = 4;
  for(float iu = jointPos[joint]; iu > 44150; iu = iu - 0.1) //Standard Speed is "0.15"
  {
  newJointPos = iu;
    if (newJointPos > jointMaxPos[joint]) 
    {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  }
  
 //OSTimeDly(1000);
  
  //Step 4
  if (step == 4)
  {
  joint = 2;
  for(float iu = jointPos[joint]; iu > 73000; iu = iu - 0.1)
  {
  newJointPos = iu;
    if (newJointPos > jointMaxPos[joint]) 
    {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  }
  //OSTimeDly(1000);
  
  //Step 5
   if (step == 5)
   {
  joint = 3;
  for(float iu = jointPos[joint]; iu < 99999; iu = iu + 0.1)
  {
  newJointPos = iu;
    if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
      
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
   }
  //OSTimeDly(500);
  
  //Step 6
  if(step == 6)
  {
   joint = 1;
    newJointPos = 80250;
    if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  //OSTimeDly(1500);
  
  //Step 7
  if (step == 7)
  {
    joint = 3;
  for(float iu = jointPos[joint]; iu < 73250; iu = iu + 0.1)
  {
  newJointPos = iu;
    if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  }
  
  //Step 7
   if (step == 8)
   {
     joint = 2;
  for(float iu = jointPos[joint]; iu > 82250; iu = iu - 0.1)
  {
  newJointPos = iu;
    if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  }
  //OSTimeDly(1000);
  

  //Step 9
     if (step == 9)
     {
  joint = 3;
  for(float iu = jointPos[joint]; iu > 74250; iu = iu - 0.1)
  {
  newJointPos = iu;
    if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
     }
  //OSTimeDly(1000);
  
  //Step 10
  if (step == 10)
  {
  joint = 4;
  for(float iu = jointPos[joint]; iu < 87250; iu = iu + 0.1)
  {
  newJointPos = iu;
    if (newJointPos > jointMaxPos[joint]) 
    {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  }
  //OSTimeDly(1000);
 
  //Step 11
  if (step == 11)
  {
  joint = 3;
  for(float iu = jointPos[joint]; iu < 86500; iu = iu + 0.1)
  {
  newJointPos = iu;
    if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  }
  //OSTimeDly(1000);
  
  //Step 12
  if (step == 12)
  {
  joint = 1;
   newJointPos = 60000;
    if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  OSTimeDly(300);
  
  //Step 13
  if (step == 13)
  {
  joint = 3;

  newJointPos = 75000;
  if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  }
  OSTimeDly(300);
  
  //Step 14
  if (step == 14)
  {
  joint = 4;
  newJointPos = 55000;
  if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
    }
    pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
  
  }
  OSTimeDly(300);
}
  

//Curent robot state
  uint32_t robotJointGetState(robotJoint_t joint) {
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  return jointPos[joint];
}

//Min Co-Ordinates
uint32_t robotJointGetMinPos(robotJoint_t joint) {
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  return jointMinPos[joint];
}

//Max Co-Ordinates
uint32_t robotJointGetMaxPos(robotJoint_t joint) {
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  return jointMaxPos[joint];
}


//Integer value to inc and Dec
uint32_t robotJointGetStepValue(void) {
  return jointStep;
}
