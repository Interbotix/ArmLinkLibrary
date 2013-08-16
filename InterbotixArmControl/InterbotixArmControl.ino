//=============================================================================
// Based upon Kurt's PX Reactor arm code.
// https://github.com/KurtE
// This code provides serial control of the Interbotix line of robotic arms, which are sold by Trossen
// Robotics: http://www.trossenrobotics.com/robotic-arms.aspx

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
#define OPT_WRISTROT          // comment this out if you are not using Wrist Rotate

#define SOUND_PIN    7      // Tell system we have added speaker to IO pin 1
#define MAX_SERVO_DELTA_PERSEC 512
//#define DEBUG             // Enable Debug mode via serial

//=============================================================================
// Global Include files
//=============================================================================
#include <ax12.h>
#include <BioloidController.h>
#include <ArmControl.h>
#include "GlobalArm.h"
//#include <Wire.h> 
//#include <LiquidCrystal_I2C.h>

//=============================================================================
//=============================================================================
// IK Modes defined, 0-4
enum {
  IKM_IK3D_CARTESIAN, IKM_IK3D_CARTESIAN_90, IKM_CYLINDRICAL, IKM_CYLINDRICAL_90, IKM_BACKHOE};

// status messages for IK return codes..
enum {
  IKS_SUCCESS=0, IKS_WARNING, IKS_ERROR};


#define IK_FUDGE            5     // How much a fudge between warning and error

//=============================================================================
// Global Objects
//=============================================================================
ArmControl armcontrol = ArmControl();
BioloidController bioloid = BioloidController(1000000);
//LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//=============================================================================
// Global Variables...
//=============================================================================
boolean         g_fArmActive = false;   // Is the arm logically on?
byte            g_bIKMode = IKM_IK3D_CARTESIAN;   // Which mode of operation are we in...
uint8_t         g_bIKStatus = IKS_SUCCESS;   // Status of last call to DoArmIK;
boolean         g_fServosFree = true;

// Current IK values
int            g_sIKGA;                  // IK Gripper angle..
int            g_sIKX;                  // Current X value in mm
int            g_sIKY;                  //
int            g_sIKZ;

// Values for current servo values for the different joints
int             g_sBase;                // Current Base servo value
int             g_sShoulder;            // Current shoulder target 
int             g_sElbow;               // Current
int             g_sWrist;               // Current Wrist value
int             g_sWristRot;            // Current Wrist rotation
int             g_sGrip;                // Current Grip position


int sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip;
int sDeltaTime = 100;


// Message informatino
unsigned long   ulLastMsgTime;          // Keep track of when the last message arrived to see if controller off
byte            buttonsPrev;            // will use when we wish to only process a button press once


//
#ifdef DEBUG
boolean        g_fDebugOutput = false;
#endif

// Forward references
extern void MSound(byte cNotes, ...);


//===================================================================================================
// Setup 
//====================================================================================================
void setup() {
  pinMode(0,OUTPUT);  
  // Lets initialize the Serial Port
  Serial.begin(38400);
  delay(500);
  Serial.println("Interbotix Robot Arm Online.");
  //
  // Next initialize the Bioloid
  bioloid.poseSize = CNT_SERVOS;

  // Read in the current positions...
  bioloid.readPose();
  delay(100);
  // Start off to put arm to sleep...
  PutArmToSleep();
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
    
    ExtArmState();  

    // See if the Arm is active yet...
    if (g_fArmActive) {

      sBase = g_sBase;
      sShoulder = g_sShoulder;
      sElbow = g_sElbow; 
      sWrist = g_sWrist;
      sWristRot = g_sWristRot;      
      sGrip = g_sGrip;

      // Call IKMode

        switch (g_bIKMode) {
        case IKM_IK3D_CARTESIAN:
          fChanged |= ProcessUserInput3D();
          break;
        case IKM_CYLINDRICAL:
          fChanged |= ProcessUserInputCylindrical();       
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
// ProcessUserInput3D: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInput3D(void) {
  
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;
  int   sIKX;                  // Current X value in mm
  int   sIKY;                  //
  int   sIKZ;
  int   sIKGA;    

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...

  if (g_bIKStatus == IKS_SUCCESS) {
    
// Keep IK values within limits
//
    sIKX = min(max(armcontrol.Xaxis, IK_MIN_X), IK_MAX_X);    
    sIKY = min(max(armcontrol.Yaxis, IK_MIN_Y), IK_MAX_Y);    
    sIKZ = min(max(armcontrol.Zaxis, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max(armcontrol.W_ang, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords..
    sWristRot = min(max(armcontrol.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armcontrol.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armcontrol.dtime*16;
    
  }

  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  



  if (fChanged) {
//    LCD(sIKX, sIKY, sIKZ, sIKGA, sWristRot, sDeltaTime);    
    g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 
  }
  return fChanged;

}


//===================================================================================================
// ProcessUserInputCylindrical: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInputCylindrical() {
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;
  int   sIKY;                  // Distance from base in mm
  int   sIKZ;
  int   sIKGA;

  // Will try combination of the other two modes.  Will see if I need to do the Limits on the IK values
  // or simply use the information from the Warning/Error from last call to the IK function...
  sIKY = g_sIKY;
  sIKZ = g_sIKZ;
  sIKGA = g_sIKGA;

  // The base rotate is real simple, just allow it to rotate in the min/max range...
  sBase = min(max((armcontrol.Xaxis+512), BASE_MIN), BASE_MAX);

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...
  // Use Y for 2d distance from base
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKY > 0) && (armcontrol.Yaxis < 0)) || ((g_sIKY < 0) && (armcontrol.Yaxis > 0)))
    sIKY = min(max(armcontrol.Yaxis, IK_MIN_Y), IK_MAX_Y);

  // Now Z coordinate...
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKZ > 0) && (armcontrol.Zaxis < 0)) || ((g_sIKZ < 0) && (armcontrol.Zaxis > 0)))
    sIKZ = min(max(armcontrol.Zaxis, IK_MIN_Z), IK_MAX_Z);

  // And gripper angle.  May leave in Min/Max here for other reasons...   

  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKGA > 0) && (armcontrol.W_ang < 0)) || ((g_sIKGA < 0) && (armcontrol.W_ang > 0)))
    sIKGA = min(max(armcontrol.W_ang, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...

    sWristRot = min(max(armcontrol.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armcontrol.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armcontrol.dtime*16;
   
  fChanged = (sBase != g_sBase) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);


  if (fChanged) {
    g_bIKStatus = doArmIK(false, sBase, sIKY, sIKZ, sIKGA);
  }
  return fChanged;
}



//===================================================================================================
// ProcessUserInputBackHoe: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInputBackHoe() {
  // lets update positions with the 4 joystick values
  // First the base
  boolean fChanged = false;
  sBase = min(max(armcontrol.Xaxis+512, BASE_MIN), BASE_MAX);
  // Now the Boom
  sShoulder = min(max(armcontrol.Yaxis, SHOULDER_MIN), SHOULDER_MAX);
  // Now the Dipper 
  sElbow = min(max(armcontrol.Zaxis, ELBOW_MIN), ELBOW_MAX);
  // Bucket Curl
  sWrist = min(max(armcontrol.W_ang, WRIST_MIN), WRIST_MAX);
    sWristRot = min(max(armcontrol.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armcontrol.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armcontrol.dtime*16;

  fChanged = (sBase != g_sBase) || (sShoulder != g_sShoulder) || (sElbow != g_sElbow) || (sWrist != g_sWrist) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  
  return fChanged;
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


#ifdef DEBUG
  if (g_fDebugOutput) {
    Serial.print("[");
    Serial.print(sBase, DEC);
    Serial.print(" ");
    Serial.print(sShoulder, DEC);
    Serial.print(" ");
    Serial.print(sElbow, DEC);
    Serial.print(" ");
    Serial.print(sWrist, DEC);
    Serial.print(" ");
    Serial.print(sWristRot, DEC);
    Serial.print(" ");
    Serial.print(sGrip, DEC);
    Serial.println("]");
  }
#endif
  // Make sure the previous movement completed.
  // Need to do it before setNextPos calls as this
  // is used in the interpolating code...
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }

  // Also lets limit how fast the servos will move as to not get whiplash.
  bioloid.setNextPose(SID_BASE, sBase);  

  sMaxDelta = abs(bioloid.getCurPose(SID_SHOULDER) - sShoulder);
  bioloid.setNextPose(SID_SHOULDER, sShoulder);

  sDelta = abs(bioloid.getCurPose(SID_ELBOW) - sElbow);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_ELBOW, sElbow);

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
// Convert radians to AX (10 bit goal address) servo position offset. 
//===================================================================================================
int radToAXServo(float rads){
  float val = (rads*100)/51 * 100;
  return (int) val;
}

//===================================================================================================
// Convert radians to MX (12 bit goal address) servo position offset. 
//===================================================================================================
int radToMXServo(float rads){
  float val = (rads*652);
  return (int) val;
}


//===================================================================================================
// Compute Arm IK for 3DOF+Mirrors+Gripper - was based on code by Michael E. Ferguson
// Hacked up by me, to allow different options...
//===================================================================================================
uint8_t doArmIK(boolean fCartesian, int sIKX, int sIKY, int sIKZ, int sIKGA)
{
  int t;
  int sol0;
  uint8_t bRet = IKS_SUCCESS;  // assume success

  if (fCartesian) {
    // first, make this a 2DOF problem... by solving baseAngle, converting to servo pos
#ifdef WIDOWX
    sol0 = radToMXServo(atan2(sIKX,sIKY));
#else
    sol0 = radToAXServo(atan2(sIKX,sIKY));
#endif    
    // remove gripper offset from base
    t = sqrt(sq((long)sIKX)+sq((long)sIKY));

    // BUGBUG... Gripper offset support
//#define G 10   
 //   sol0 -= radToServo(atan2((G/2)-G_OFFSET,t));
 
  }
  else {
    // We are in cylindrical mode, probably simply set t to the y we passed in...
    t = sIKY;
#ifdef DEBUG
    sol0 = 0;
#endif
  }
  // convert to sIKX/sIKZ plane, remove wrist, prepare to solve other DOF           
  float flGripRad = (float)(sIKGA)*3.14159/180.0;
  long trueX = t - (long)((float)WristLength*cos(flGripRad));   
  long trueZ = sIKZ - BaseHeight - (long)((float)WristLength*sin(flGripRad));

  long im = sqrt(sq(trueX)+sq(trueZ));        // length of imaginary arm
  float q1 = atan2(trueZ,trueX);              // angle between im and X axis
  long d1 = sq(ShoulderLength) - sq(ElbowLength) + sq((long)im);
  long d2 = 2*ShoulderLength*im;
  float q2 = acos((float)d1/float(d2));
  q1 = q1 + q2;

//  int sol1 = radToServo(q1-1.57);  

  d1 = sq(ShoulderLength)-sq((long)im)+sq(ElbowLength);
  d2 = 2*ElbowLength*ShoulderLength;
  q2 = acos((float)d1/(float)d2);

#ifdef WIDOWX  
  int sol1 = radToMXServo(q1-1.57);
  int sol2 = radToMXServo(3.14-q2);
  // solve for wrist rotate
  int sol3 = radToMXServo(3.2 + flGripRad - q1 - q2 );
#else
  int sol1 = radToAXServo(q1-1.57);
  int sol2 = radToAXServo(3.14-q2);
  // solve for wrist rotate
  int sol3 = radToAXServo(3.2 + flGripRad - q1 - q2 );
#endif  

    // Lets calculate the actual servo values.

  if (fCartesian) {
    sBase = min(max(2048 - sol0, BASE_MIN), BASE_MAX);
  }
  sShoulder = min(max(2048 - sol1, SHOULDER_MIN), SHOULDER_MAX);

  // Magic Number 819???
  sElbow = min(max(3072 - sol2, ELBOW_MIN), ELBOW_MAX);

#define Wrist_Offset 2048
  sWrist = min(max(Wrist_Offset + sol3, WRIST_MIN), WRIST_MAX);

  // Remember our current IK positions
  g_sIKX = sIKX; 
  g_sIKY = sIKY;
  g_sIKZ = sIKZ;
  g_sIKGA = sIKGA;
  // Simple test im can not exceed the length of the Shoulder+Elbow joints...

  if (im > (ShoulderLength + ElbowLength)) {
    if (g_bIKStatus != IKS_ERROR) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        Serial.println("IK Error");
      }
#endif
      MSound(2, 50, 3000, 50, 3000);
    }
    bRet = IKS_ERROR;  
  }
  else if(im > (ShoulderLength + ElbowLength-IK_FUDGE)) {
    if (g_bIKStatus != IKS_WARNING) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        Serial.println("IK Warning");
      }
#endif
      MSound(1, 75, 2500);
    }
    bRet = IKS_WARNING;  
  }

  return bRet;
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




void IDPacket()  {
  Serial.write(0xFF);
  Serial.write((unsigned char) ARMID);
  Serial.write((unsigned char) g_bIKMode);
  Serial.write((unsigned char) 0);
  Serial.write((unsigned char)(255 - (ARMID+g_bIKMode+0)%256));
}




//===================================================================================================
// Check EXT packet to determine action
//===================================================================================================
void ExtArmState(){
       if(armcontrol.ext < 0x10){
        // no action
        g_fArmActive = true;
     }
    switch (armcontrol.ext){
      case 0x20:  //32
        g_bIKMode = IKM_IK3D_CARTESIAN;
        MoveArmToHome(); 
        IDPacket();
        break;
      case 0x28:  //40
        g_bIKMode = IKM_IK3D_CARTESIAN_90;
        MoveArmTo90Home(); 
        IDPacket();
        break;        
      case 0x30:  //48
        g_bIKMode = IKM_CYLINDRICAL;
        MoveArmToHome(); 
        IDPacket();        
        break;    
      case 0x38:  //56
        g_bIKMode = IKM_CYLINDRICAL_90;
        MoveArmTo90Home(); 
        IDPacket();        
        break;         
      case 0x40:  //64
        g_bIKMode = IKM_BACKHOE;
        MoveArmToHome(); 
        IDPacket();        
        break;
      case 0x50:  //80
        MoveArmToHome(); 
        IDPacket();        
        break;
      case 0x60:  //96
        PutArmToSleep();
        IDPacket();        
        break;
      case 0x70:  //112
        //do something
        break;
      case 0x80:  //128
        //do something
        break;        
      case 0x90:  //144
        //do something
        break;
    }
}




//void LCD(int IKX, int IKY, int IKZ, int IKGA, int WristRot, int Gripper){
//    lcd.setCursor(0, 0);    
//    lcd.print(IKX);    
//    lcd.setCursor(4, 0);    
//    lcd.print(IKY);      
//    lcd.setCursor(8, 0);    
//    lcd.print(IKZ);    
//    lcd.setCursor(12, 0);    
//    lcd.print(IKGA);
//    lcd.setCursor(0, 1);    
//    lcd.print(WristRot);    
//    lcd.setCursor(8, 1);    
//    lcd.print(Gripper); 
//}

