#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "Kinematics.h"
#include "GlobalArm.h"

extern ArmControl armcontrol;

extern void MoveArmToHome(void);
extern void MoveArmTo90Home(void);
extern void IDPacket(void);
extern void PutArmToSleep(void);

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
      case 0x58: //88
        MoveArmTo90Home();
        IDPacket();
        break;  
      case 0x60:  //96
        PutArmToSleep();
        IDPacket();        
        break;
      case 0x70:  //112
        IDPacket();
        break;
      case 0x80:  //128
        //IK value response
        break;        
      case 0x90:  //144
        //do something
        break;
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
    sIKX = min(max((armcontrol.Xaxis-X_OFFSET), IK_MIN_X), IK_MAX_X);  
    sIKY = min(max(armcontrol.Yaxis, IK_MIN_Y), IK_MAX_Y);    
    sIKZ = min(max(armcontrol.Zaxis, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max((armcontrol.W_ang-GA_OFFSET), IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords..
    sWristRot = min(max(armcontrol.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armcontrol.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armcontrol.dtime*16;
    
  }

  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  
  
  if (fChanged) {
    // report
    g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 
  }
  return fChanged;

}


//===================================================================================================
// ProcessUserInput3D90: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInput3D90(void) {
  
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
    sIKX = min(max((armcontrol.Xaxis-X_OFFSET), IK_MIN_X_90), IK_MAX_X_90);  
    sIKY = min(max(armcontrol.Yaxis, IK_MIN_Y_90), IK_MAX_Y_90);    
    sIKZ = min(max(armcontrol.Zaxis, IK_MIN_Z_90), IK_MAX_Z_90);
    sIKGA = min(max((armcontrol.W_ang-GA_OFFSET), IK_MIN_GA_90), IK_MAX_GA_90);  // Currently in Servo coords..
    sWristRot = min(max(armcontrol.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armcontrol.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armcontrol.dtime*16;
    
  }

  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  
  
  if (fChanged) {
    // report
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
  sBase = min(max(armcontrol.Xaxis, BASE_MIN), BASE_MAX);

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
    sIKGA = min(max((armcontrol.W_ang-GA_OFFSET), IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...

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
// ProcessUserInputCylindrical90: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInputCylindrical90() {
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
  sBase = min(max(armcontrol.Xaxis, BASE_MIN), BASE_MAX);

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...
  // Use Y for 2d distance from base
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKY > 0) && (armcontrol.Yaxis < 0)) || ((g_sIKY < 0) && (armcontrol.Yaxis > 0)))
    sIKY = min(max(armcontrol.Yaxis, IK_MIN_Y_90), IK_MAX_Y_90);

  // Now Z coordinate...
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKZ > 0) && (armcontrol.Zaxis < 0)) || ((g_sIKZ < 0) && (armcontrol.Zaxis > 0)))
    sIKZ = min(max(armcontrol.Zaxis, IK_MIN_Z_90), IK_MAX_Z_90);

  // And gripper angle.  May leave in Min/Max here for other reasons...   

  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKGA > 0) && (armcontrol.W_ang < 0)) || ((g_sIKGA < 0) && (armcontrol.W_ang > 0)))
    sIKGA = min(max((armcontrol.W_ang-GA_OFFSET), IK_MIN_GA_90), IK_MAX_GA_90);  // Currently in Servo coords...

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
  sBase = min(max(armcontrol.Xaxis, BASE_MIN), BASE_MAX);
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







//=============================================================================
//=============================================================================
#endif
