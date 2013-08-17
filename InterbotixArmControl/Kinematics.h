#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "GlobalArm.h"
//#include "InputControl.h"

// Forward references
extern void MSound(byte cNotes, ...);

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
    sBase = min(max(BASE_N - sol0, BASE_MIN), BASE_MAX);
  }
  sShoulder = min(max(SHOULDER_N - sol1, SHOULDER_MIN), SHOULDER_MAX);

  sElbow = min(max(ELBOW_N - sol2, ELBOW_MIN), ELBOW_MAX);

  sWrist = min(max(WRIST_N + sol3, WRIST_MIN), WRIST_MAX);

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





















//=============================================================================
//=============================================================================
#endif
