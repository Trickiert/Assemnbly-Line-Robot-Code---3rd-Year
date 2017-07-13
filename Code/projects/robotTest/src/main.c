/*
 * Simple uC/OS-II program to flash the LEDs while sending CAN messages on port 1
 * and receiving them in an interrupt fashion on port 2.
 *
 * DK - 17-Nov 2010
 */


#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <lcd.h>
#include <can.h>
#include <buttons.h>
#include <interface.h>
#include <robot.h>

/*
*********************************************************************************************************
*                                            APPLICATION TASK PRIORITIES
*********************************************************************************************************
*/

enum {
  APP_TASK_LINK_PRIO = 4,
  APP_TASK_CONNECT_PRIO,
  APP_TASK_CAN_WRITE_PRIO,
  APP_TASK_CAN_READ_PRIO
};

/*
*********************************************************************************************************
*                                            APPLICATION TASK STACKS
*********************************************************************************************************
*/

#define  APP_TASK_LINK_STK_SIZE                 256
#define  APP_TASK_CONNECT_STK_SIZE              256
#define  APP_TASK_CAN_WRITE_STK_SIZE            256
#define  APP_TASK_CAN_READ_STK_SIZE             256


static OS_STK appTaskLinkStk[APP_TASK_LINK_STK_SIZE];
static OS_STK appTaskConnectStk[APP_TASK_CONNECT_STK_SIZE];
static OS_STK appTaskCANWriteStk[APP_TASK_CAN_WRITE_STK_SIZE];
static OS_STK appTaskCANReadStk[APP_TASK_CAN_READ_STK_SIZE];

/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/

uint32_t rxCount;                       // counter for CAN message receptions
uint32_t txCount;                       // counter for CAN message transmissions
canMessage_t can1RxBuffer;              // CAN message receive buffer for CAN port 1
canMessage_t can2RxBuffer;              // CAN message receive buffer for CAN port 1
bool can1RxDone;                        // Flag indicating message reception complete on CAN port 1
bool can2RxDone;                        // Flag indicating message reception complete on CAN port 1

bool podiumSensor = false;
bool beltSensor = false;

bool pause = false;
bool stop = false;
bool emergencyStop = false;
bool blockInClamp = false;

/*
*********************************************************************************************************
*                                            APPLICATION FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void appTaskLink(void *pdata);
static void appTaskConnect(void *pdata);
static void appTaskCANWrite(void *pdata);
static void appTaskCANRead(void *pdata);

/*
*********************************************************************************************************
*                                            LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void canHandler(void);                   // interrupt handler for CAN message reception

/*
*********************************************************************************************************
*                                            GLOBAL FUNCTION DEFINITIONS
*********************************************************************************************************
*/

int main() {

  /* Initialise the board support package */
  bspInit();
  robotInit();

  /* Initialise the OS */
  OSInit();                                                   

  /* Create the tasks */
  OSTaskCreate(appTaskLink,                               
               (void *)0,
               (OS_STK *)&appTaskLinkStk[APP_TASK_LINK_STK_SIZE - 1],
               APP_TASK_LINK_PRIO);

  OSTaskCreate(appTaskConnect,                               
               (void *)0,
               (OS_STK *)&appTaskConnectStk[APP_TASK_CONNECT_STK_SIZE - 1],
               APP_TASK_CONNECT_PRIO);

  OSTaskCreate(appTaskCANWrite,                               
               (void *)0,
               (OS_STK *)&appTaskCANWriteStk[APP_TASK_CAN_WRITE_STK_SIZE - 1],
               APP_TASK_CAN_WRITE_PRIO);

  OSTaskCreate(appTaskCANRead,                               
               (void *)0,
               (OS_STK *)&appTaskCANReadStk[APP_TASK_CAN_READ_STK_SIZE - 1],
               APP_TASK_CAN_READ_PRIO);
  
  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*
*********************************************************************************************************
*                                            APPLICATION TASK DEFINITIONS
*********************************************************************************************************
*/

static void appTaskLink(void *pdata) {
  /* Start the OS ticker 
   * Must be done in the highest priority task
   */
  osStartTick();
  
  /* Task main loop */
  while (true) {
    ledToggle(USB_LINK_LED);
    OSTimeDlyHMSM(0,0,0,500);
    
          static bool jsRightPressed = false;
          uint32_t btnState;
   
             //Isolated Test
        btnState = buttonsRead();
        if (isButtonPressedInState(btnState, JS_RIGHT)) 
	{       
        beltSensor = true;
        }

  }
}

static void appTaskConnect(void *pdata) {
  
  while (true) {
    OSTimeDlyHMSM(0,0,0,500);
    ledToggle(USB_CONNECT_LED);      

  
  } 
}


static void appTaskCANWrite(void *pdata) {
  
  canMessage_t msg;
  bool txOk;
  bool two = false;
  bool one = false;
  int starts = 1;
  
  msg.id = 0x07;  // arbitrary CAN message id
  msg.len = 8;    // data length 4
  msg.dataA = 0;
  msg.dataB = 10;
  
  while ( true ) {
    // Transmit message on CAN 1
    txOk = canWrite(CAN_PORT_1, &msg);
    if (txOk) {
      txCount += 1;
      msg.dataA = txCount;
      msg.dataB = txCount;
      if(two == true && one == true)
      {
        msg.dataA = 1;
      }
      else if(two == false && one == true)
      {
        msg.dataA = 2;
      }
      else if(two == false && one == false)
      {
        msg.dataA = 3;
        one = true;
      }
      
    }
    OSTimeDly(250);
  }
}


static void appTaskCANRead(void *pdata) {
  

  canRxInterrupt(canHandler);    // configure CAN to interrupt on message reception
  static uint8_t joint = ROBOT_HAND;
  int step = 1;
  
  while ( true ) {
    if (can2RxDone) {           // could be handled better with a semaphore
      can2RxDone = false;
      lcdSetTextPos(2,1);
      lcdWrite("ID    : %x", can2RxBuffer.id); 
      lcdSetTextPos(2,2);
      lcdWrite("LEN   : %x", can2RxBuffer.len); 
      lcdSetTextPos(2,3);
      lcdWrite("DATA_A: %x", can2RxBuffer.dataA); 
      lcdSetTextPos(2,4);
      lcdWrite("DATA_B: %x", can2RxBuffer.dataB);
      lcdSetTextPos(2,5);
     
      if(can2RxBuffer.dataB == 1)
      {
        pause = false;
        stop = false;
      }
      if(can2RxBuffer.dataB == 2)
      {
        pause = true;
      }
      if(can2RxBuffer.dataB == 3)
      {
        stop = true;
      }
      if(can2RxBuffer.dataB == 4)
      {
        emergencyStop = true;
      }
      
        if(can2RxBuffer.dataA == 6 || can2RxBuffer.dataB == 6 || can2RxBuffer.dataA == 8 || can2RxBuffer.dataB == 8 )
      {
        beltSensor = true;
      }
        
      if(pause == false && emergencyStop == false && stop == false)
      {
        if(can2RxBuffer.dataA == 6 && step == 1 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 2;
        blockInClamp = true;
        }
        if(blockInClamp == true && step == 2 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 3;
        }
        if(blockInClamp == true && step == 3 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 4;
        }
        if(blockInClamp == true && step == 4 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 5;
        }
        if(blockInClamp == true && step == 5 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 6;
        }
        if(blockInClamp == true && step == 6 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 7;
        }
        if(blockInClamp == true && step == 7 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 8;
        }
        if(blockInClamp == true && step == 8 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 9;
        }
        if(blockInClamp == true && step == 9 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 10;
        }
        if(blockInClamp == true && step == 10 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 11;
        //blockInClamp = false;
        }
        
         if(blockInClamp == true && step == 11 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 12;
        //blockInClamp = false;
        }
        
         if(blockInClamp == true && step == 12 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 13;
        }
        
         if(blockInClamp == true && step == 13 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 14;
        //blockInClamp = false;
        }
        
         if(blockInClamp == true && step == 14 && can2RxBuffer.dataB != 4)
        {
        joint = 1;
        robotJointSetState((robotJoint_t)joint, step);
        step = 1;
        blockInClamp = false; //End
        }
      }
    }
    OSTimeDly(100);
  }
}

/*
 * A simple interrupt handler for CAN message reception
 */
void canHandler(void) {
  if (canReady(CAN_PORT_1)) {
    canRead(CAN_PORT_1, &can1RxBuffer);
    can1RxDone = true;
  }
  if (canReady(CAN_PORT_2)) {
    canRead(CAN_PORT_2, &can2RxBuffer);
    can2RxDone = true;
  }
}