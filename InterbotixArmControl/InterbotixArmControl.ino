//=============================================================================
// Based upon Kurt's PX Reactor arm code.
// https://github.com/KurtE
// This code provides serial control of the Interbotix line of robotic arms, which are sold by Trossen
// Robotics: http://www.trossenrobotics.com/robotic-arms.aspx
// http://learn.trossenrobotics.com/interbotix/robot-arms
//=============================================================================
//Armcontrol packet structure is as follows
//
// 255, XH, XL, YH, YL, ZH, ZL, WAH, WAL, WRH, WAL, GH, GL, DTIME, BUTTONS, EXT, CHECKSUM
//
// (please note, actual XYZ min/max for specific arm defined below)
//
// Protocol value ranges
//
// XH = high byte X-axis
// XL = low byte, 0-1023 (-512 through +512 via ArmControl)
//
// YH = high byte Y-axis
// YL = low byte, 0-1023
//
// ZH = high byte Z-axis
// ZL = low byte, 0-1023 
//
// WAH = high byte (unused for now, placeholder for higher res wrist angle)
// WAL = low byte, 0-180 (-90 through +90 via ArmControl)
//
// WRH = high byte 
// WRL = low byte, 0-1023. 512 center
//
// GH = high byte
// GL = low byte, 0-512. 256 center
//
// DTIME = byte. DTIME*16 = interpolation delta time
//
// Buttons = byte (not implemented)
//
// EXT = byte. Extended instruction set.
// EXT < 16 = no action
// EXT = 32 = 3D Cartesian IK
// EXT = 48 = Cylindrical IK Xaxis = 0-4096 value, untested
// EXT = 64 = BackHoe aka passthrough UNTESTED
//
// CHECKSUM = (unsigned char)(255 - (XH+XL+YH+YL+ZH+ZL+WAH+WAL+WRH+WRL+GH+GL+DTIME+BUTTONS+EXT)%256)

//  This code is a Work In Progress and is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
//  
//=============================================================================

//=============================================================================
// Define Options
//=============================================================================

//#define NOPE
//#define PINCHER
//#define REACTOR
#define WIDOWX


#define SOUND_PIN    7      // Tell system we have added speaker to IO pin 1
#define MAX_SERVO_DELTA_PERSEC 512
//#define DEBUG             // Enable Debug mode via serial

//=============================================================================
// Global Include files
//=============================================================================
#include <ax12.h>
#include <BioloidController.h>
#include <ArmControl.h>
#include "InputControl.h"

//=============================================================================
// Global Objects
//=============================================================================
BioloidController bioloid = BioloidController(1000000);
ArmControl armcontrol = ArmControl();



// Message informatino
unsigned long   ulLastMsgTime;          // Keep track of when the last message arrived to see if controller off
byte            buttonsPrev;            // will use when we wish to only process a button press once



//===================================================================================================
// Setup 
//====================================================================================================
void setup() {
  //Serial activity LED output
  pinMode(0,OUTPUT);  
  // Lets initialize the Serial Port
  Serial.begin(38400);
  delay(500);
  Serial.println("Interbotix Robot Arm Online.");

  // Next initialize the Bioloid
  bioloid.poseSize = CNT_SERVOS;

  // Read in the current positions...
  bioloid.readPose();
  delay(100);
  // Start off to put arm to sleep...
  PutArmToSleep();
  //Send ID Packet
  IDPacket();
  MSound(3, 60, 2000, 80, 2250, 100, 2500);

  //set Gripper Compliance so it doesn't tear itself apart
  ax12SetRegister(SID_GRIP, AX_CW_COMPLIANCE_SLOPE, 128);
  ax12SetRegister(SID_GRIP, AX_CCW_COMPLIANCE_SLOPE, 128);

}


//===================================================================================================
// loop: Our main Loop!
//===================================================================================================
void loop() {
  boolean fChanged = false;
   
  if (armcontrol.ReadMsgs()) {
  
    digitalWrite(0,HIGH-digitalRead(0));    
    
    // Check EXT packet to determine action. Found in InputControl.h
    ExtArmState();  

    // See if the Arm is active yet...
    if (g_fArmActive) {

      sBase = g_sBase;
      sShoulder = g_sShoulder;
      sElbow = g_sElbow; 
      sWrist = g_sWrist;
      sWristRot = g_sWristRot;      
      sGrip = g_sGrip;

      // Set InputControl function based on which IKMode we're in

        switch (g_bIKMode) {
        case IKM_IK3D_CARTESIAN:
          fChanged |= ProcessUserInput3D();
          break;
        case IKM_IK3D_CARTESIAN_90:
          fChanged |= ProcessUserInput3D90();
          break;          
        case IKM_CYLINDRICAL:
          fChanged |= ProcessUserInputCylindrical();       
          break;
        case IKM_CYLINDRICAL_90:
          fChanged |= ProcessUserInputCylindrical90();       
          break;
        case IKM_BACKHOE:
          fChanged |= ProcessUserInputBackHoe();
          break;
        }




      // If something changed and we are not in an error condition
      if (fChanged && (g_bIKStatus != IKS_ERROR)) {
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);
      }
      else if (bioloid.interpolating > 0) {
        bioloid.interpolateStep();
      }
    }
//    buttonsPrev = armcontrol.buttons;

  }
  else {
    if (bioloid.interpolating > 0) {
      bioloid.interpolateStep();
    }
  }
} 




//===================================================================================================
// MoveArmTo
//===================================================================================================
void MoveArmTo(int sBase, int sShoulder, int sElbow, int sWrist, int sWristRot, int sGrip, int wTime, boolean fWait) {

  int sMaxDelta;
  int sDelta;

  // First make sure servos are not free...
  if (g_fServosFree) {
    g_fServosFree = false;

    for(uint8_t i=1; i <= CNT_SERVOS; i++) {
      TorqueOn(i);
    }  
    delay(100);    //required due to power dip on TorqueOn
    bioloid.readPose();
  }


  // Make sure the previous movement completed.
  // Need to do it before setNextPos calls as this
  // is used in the interpolating code...
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }

  // Also lets limit how fast the servos will move as to not get whiplash.
  bioloid.setNextPose(SID_BASE, sBase);  

#ifdef REACTOR
  sMaxDelta = abs(bioloid.getCurPose(SID_RSHOULDER) - sShoulder);
  bioloid.setNextPose(SID_RSHOULDER, sShoulder);
  bioloid.setNextPose(SID_LSHOULDER, 1024-sShoulder);

  sDelta = abs(bioloid.getCurPose(SID_RELBOW) - sElbow);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_RELBOW, sElbow);
  bioloid.setNextPose(SID_LELBOW, 1024-sElbow);
  
#else
  sMaxDelta = abs(bioloid.getCurPose(SID_SHOULDER) - sShoulder);
  bioloid.setNextPose(SID_SHOULDER, sShoulder);

  sDelta = abs(bioloid.getCurPose(SID_ELBOW) - sElbow);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_ELBOW, sElbow);
#endif

  sDelta = abs(bioloid.getCurPose(SID_WRIST) - sWrist);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_WRIST, sWrist);

#ifdef OPT_WRISTROT
  bioloid.setNextPose(SID_WRISTROT, sWristRot); 
#endif  

  bioloid.setNextPose(SID_GRIP, sGrip);


  // Save away the current positions...
  g_sBase = sBase;
  g_sShoulder = sShoulder;
  g_sElbow = sElbow;
  g_sWrist = sWrist;
  g_sWristRot = sWristRot;
  g_sGrip = sGrip;



  // Now start the move - But first make sure we don't move too fast.  
//  if (((long)sMaxDelta*wTime/1000L) > MAX_SERVO_DELTA_PERSEC) {
//    wTime = ((long)sMaxDelta*1000L)/ MAX_SERVO_DELTA_PERSEC;
//  }

  bioloid.interpolateSetup(wTime);

  // Do at least the first movement
  bioloid.interpolateStep();

  // And if asked to, wait for the previous move to complete...
  if (fWait) {
    while (bioloid.interpolating > 0) {
      bioloid.interpolateStep();
      delay(3);
    }
  }
}


//===================================================================================================
// MoveArmToHome
//===================================================================================================
void MoveArmToHome(void) {
//  if (g_bIKMode != IKM_BACKHOE) {
    g_bIKStatus = doArmIK(true, 0, (2*ElbowLength)/3+WristLength, BaseHeight+(2*ShoulderLength)/3, 0);
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 256, 2000, true);
     g_fArmActive = false;
//  }
//  else {
//    g_bIKStatus = IKS_SUCCESS;  // assume sucess soe we will continue to move...
//    MoveArmTo(2048, 2048, 2048, 2048, 512, 256, 2000, true);
//  }
}

//===================================================================================================
// MoveArmTo90Home
//===================================================================================================
void MoveArmTo90Home(void) {

//  if (g_bIKMode != IKM_BACKHOE) {
    g_bIKStatus = doArmIK(true, 0, 150, 30, -90);
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 256, 2000, true);
     g_fArmActive = false;
//  }
//  else {
//    g_bIKStatus = IKS_SUCCESS;  // assume sucess soe we will continue to move...
//    MoveArmTo(2048, 2048, 2048, 2048, 512, 256, 2000, true);
//  }
}

//===================================================================================================
// PutArmToSleep
//===================================================================================================
void PutArmToSleep(void) {
  g_fArmActive = false;
  MoveArmTo(2048, 1024, 1024, 1700, 512, 256, 2000, true);

  // And Relax all of the servos...
  for(uint8_t i=1; i <= CNT_SERVOS; i++) {
    Relax(i);
  }
  g_fServosFree = true;
}


void IDPacket()  {
  Serial.write(0xFF);
  Serial.write((unsigned char) ARMID);
  Serial.write((unsigned char) g_bIKMode);
  Serial.write((unsigned char) 0);
  Serial.write((unsigned char)(255 - (ARMID+g_bIKMode+0)%256));
}





// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
#ifdef SOUND_PIN
void SoundNoTimer(unsigned long duration,  unsigned int frequency)
{
#ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
#else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
#endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask = digitalPinToBitMask(SOUND_PIN);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    *pin_port ^= pin_mask;

    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  *pin_port &= ~(pin_mask);  // keep pin low after stop

}

void MSound(byte cNotes, ...)
{
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}
#else
void MSound(byte cNotes, ...)
{
};
#endif




