#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "Kinematics.h"
#include "GlobalArm.h"

extern ArmLink armlink;

extern void MoveArmToHome(void);
extern void MoveArmTo90Home(void);
extern void IDPacket(void);
extern void PutArmToSleep(void);
extern void ReportAnalog(unsigned char, unsigned int);



//=============================================================================
// DIGITAL INPUT CONFIG...
//=============================================================================

// use #define to set the I/O numbers, since these will never change - this saves us memory while the Arduino is running
#define BUTTON1 1
#define BUTTON2 2


//variables to hold the current status of the button.(LOW == unpressed, HIGH = pressed)
int buttonState1 = LOW;         
int buttonState2 = LOW;            
boolean   loopbreak = LOW;


//////////////////////////////////////////////////////////////////////////////
// ANALOG INPUT CONFIG  // 
//////////////////////////////////////////////////////////////////////////////
//define analog pins that will be connected to the joystick pins
#define ANALOGX       0  //connected to Horizontal Axis on Joystick # 1
#define ANALOGY       1  //connected to Vertical Axis on Joystick # 2
#define ANALOGZ       2  //connected to Vertical Axis on Joystick # 3
#define ANALOGGA      4  //connected to Vertical Axis on Joystick # 4
#define ANALOGWR      3  //connected to Vertical Axis on Joystick # 4
#define ANALOGGRIP    5  //connected to Rotation Knob / Potentiometer # 1

//generic deadband limits - not all joystics will center at 512, so these limits remove 'drift' from joysticks that are off-center.
#define DEADBANDLOW 492   //decrease this value if drift occurs, increase it to increase sensitivity around the center position
#define DEADBANDHIGH 532  //increase this value if drift occurs, decrease it to increase sensitivity around the center position


 //last read values of analog sensors (Native values, 0-1023)
int joyXVal = 0;     //present value of the base rotation knob (analog 0)
int joyYVal = 0; //present value of the shoulder joystick (analog 1)
int joyZVal = 0;    //present value of the elbow joystick (analog 2)
int joyGAVal = 0;    //present value of the wrist joystick (analog 3)
int joyWRVal = 0;    //present value of the wrist joystick (analog 3)
int joyGripperVal = 0;  //present value of the gripper rotation knob (analog 4)

//last calculated values of analog sensors (Mapped values)
//knob values (base and gripper) will be mapped directly to the servo limits
//joystick values (shoulder, elbow and wrist) will be mapped from -spd to spd, to faciliate incremental control
int joyXMapped = 0;      //base knob value, mapped from 1-1023 to BASE_MIN-BASE_MAX
int joyYMapped = 0;  //shoulder joystick value, mapped from 1-1023 to -spd to spd
int joyZMapped = 0;     //elbow joystick value, mapped from 1-1023 to -spd to spd
int joyGAMapped = 0;     //wrist joystick value, mapped from 1-1023 to -spd to spd
int joyWRMapped = 0;     //wrist joystick value, mapped from 1-1023 to -spd to spd
int joyGripperMapped = 0;   //gripper knob  value, mapped from 1-1023 to GRIPPER_MIN-GRIPPER_MAX

int spd = 5;  //speed modififer, increase this to increase the speed of the movement


//===================================================================================================
// Check EXT packet to determine action
//===================================================================================================
   void ExtArmState(){
       if(armlink.ext < 0x10){
        // no action
        g_fArmActive = true;
     }
      else if(armlink.ext == 0x20){  //32
        g_bIKMode = IKM_IK3D_CARTESIAN;
        MoveArmToHome(); 
        IDPacket();
      }
      else if(armlink.ext == 0x28){  //40
        g_bIKMode = IKM_IK3D_CARTESIAN_90;
        MoveArmTo90Home(); 
        IDPacket();
      }        
      else if(armlink.ext == 0x30){  //48
        g_bIKMode = IKM_CYLINDRICAL;
        MoveArmToHome(); 
        IDPacket();        
      }
      else if(armlink.ext == 0x38){  //56
        g_bIKMode = IKM_CYLINDRICAL_90;
        MoveArmTo90Home(); 
        IDPacket();        
      }        
      else if(armlink.ext == 0x40){  //64
        g_bIKMode = IKM_BACKHOE;
        MoveArmToHome(); 
        IDPacket();        
      }
      else if(armlink.ext == 0x48){  //72
      // do something
      }
      else if(armlink.ext == 0x50){  //80
        MoveArmToHome(); 
        IDPacket();        
      }
      else if(armlink.ext == 0x58){  //88
        MoveArmTo90Home();
        IDPacket();
      }
      else if(armlink.ext == 0x60){  //96
        PutArmToSleep();
        IDPacket();        
      }
      else if(armlink.ext == 0x70){  //112
        IDPacket();
      }
      else if(armlink.ext == 0x80){  //128
        //IK value response
      }
      else if(armlink.ext >= 0xC8){  //200
        // read analogs
        ReportAnalog(armlink.ext, analogRead(armlink.ext - 0xC8));
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
//    sIKX = min(max((armlink.Xaxis-X_OFFSET), IK_MIN_X), IK_MAX_X);  
//    sIKY = min(max(armlink.Yaxis, IK_MIN_Y), IK_MAX_Y);    
//    sIKZ = min(max(armlink.Zaxis, IK_MIN_Z), IK_MAX_Z);
//    sIKGA = min(max((armlink.W_ang-GA_OFFSET), IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords..
    sWristRot = min(max(armlink.W_rot, WROT_MIN), WROT_MAX);
    sDeltaTime = armlink.dtime*16;
    
  }

  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  
  
  if (fChanged) {
    // report
    g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 
  }
  return fChanged;

}


//===================================================================================================
// ProcessUserInput3D: Process the analog input  when we are in 3d Mode
//===================================================================================================
boolean ProcessAnalogInput3D(void) {
  
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
  
  //read analog values from analog sensors
  joyXVal = analogRead(ANALOGX);
  joyYVal = analogRead(ANALOGY);
  joyZVal = analogRead(ANALOGZ);
  joyGAVal = analogRead(ANALOGGA);
  joyWRVal = analogRead(ANALOGWR);
  joyGripperVal = analogRead(ANALOGGRIP);
  delay(5);


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
  {
    joyXMapped = map(joyXVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKX -= joyXMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
  {
    joyYMapped = map(joyYVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKY -= joyYMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
  {
    joyZMapped = map(joyZVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKZ -= joyZMapped;
  }


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
  {
    joyGAMapped = map(joyGAVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKGA -= joyGAMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyWRVal > DEADBANDHIGH || joyWRVal < DEADBANDLOW)
  {
    joyWRMapped = map(joyWRVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sWristRot += joyWRMapped;
  }

  //Mapping analog joystick value to servo  signal range
  joyGripperMapped = map(joyGripperVal, 0, 1023, GRIP_MIN, GRIP_MAX);
  sGrip = joyGripperMapped;//set servo position variable to the mapped value from the knob



  sIKX = min(max(g_sIKX, IK_MIN_X), IK_MAX_X);
  sIKY = min(max(g_sIKY, IK_MIN_Y), IK_MAX_Y);
  sIKZ = min(max(g_sIKZ, IK_MIN_Z), IK_MAX_Z);
  sIKGA = min(max(g_sIKGA, IK_MIN_GA), IK_MAX_GA);  
  sWristRot = min(max(g_sWristRot, WROT_MIN), WROT_MAX);

  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  


  g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 

  return fChanged;

}



//===================================================================================================
// ProcessUserInput3D90: Process the Analog input when we are in 3d Mode, 90 degree
//===================================================================================================
boolean ProcessAnalogInput3D90(void) {
  
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
  
  //read analog values from analog sensors
  joyXVal = analogRead(ANALOGX);
  joyYVal = analogRead(ANALOGY);
  joyZVal = analogRead(ANALOGZ);
  joyGAVal = analogRead(ANALOGGA);
  joyWRVal = analogRead(ANALOGWR);
  joyGripperVal = analogRead(ANALOGGRIP);
  delay(5);


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
  { 
    joyXMapped = map(joyXVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKX -= joyXMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
  {
    joyYMapped = map(joyYVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKY -= joyYMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
  {
    joyZMapped = map(joyZVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKZ -= joyZMapped;
  }


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
  {
    joyGAMapped = map(joyGAVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKGA -= joyGAMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyWRVal > DEADBANDHIGH || joyWRVal < DEADBANDLOW)
  {
    joyWRMapped = map(joyWRVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sWristRot += joyWRMapped;
  }

  //Mapping analog joystick value to servo  signal range
  joyGripperMapped = map(joyGripperVal, 0, 1023, GRIP_MIN, GRIP_MAX);
  sGrip = joyGripperMapped;//set servo position variable to the mapped value from the knob



  sIKX = min(max(g_sIKX,IK_MIN_X_90), IK_MAX_X_90);  
  sIKY = min(max(g_sIKY, IK_MIN_Y_90), IK_MAX_Y_90);    
  sIKZ = min(max(g_sIKZ, IK_MIN_Z_90), IK_MAX_Z_90);
  sIKGA = min(max((g_sIKGA - GA_OFFSET), IK_MIN_GA_90), IK_MAX_GA_90); 
  sWristRot = min(max(g_sWristRot, WROT_MIN), WROT_MAX);

  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  


  g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 

  return fChanged;


}

//===================================================================================================
// ProcessUserInputCylindrical: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessAnalogInputCylindrical() {
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
  //int   sIKX;                  // Current X value in mm
  int   sIKY;                  //
  int   sIKZ;
  int   sIKGA;    

  // Will try combination of the other two modes.  Will see if I need to do the Limits on the IK values
  // or simply use the information from the Warning/Error from last call to the IK function...
  sBase = g_sBase;
  sIKY = g_sIKY;
  sIKZ = g_sIKZ;
  sIKGA = g_sIKGA;
  
  //read analog values from analog sensors
  joyXVal = analogRead(ANALOGX);
  joyYVal = analogRead(ANALOGY);
  joyZVal = analogRead(ANALOGZ);
  joyGAVal = analogRead(ANALOGGA);
  joyWRVal = analogRead(ANALOGWR);
  joyGripperVal = analogRead(ANALOGGRIP);
  delay(5);


  


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
  {
    joyXMapped = map(joyXVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sBase += joyXMapped;
  }


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
  {
    joyYMapped = map(joyYVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKY -= joyYMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
  {
    joyZMapped = map(joyZVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKZ -= joyZMapped;
  }


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
  {
    joyGAMapped = map(joyGAVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKGA -= joyGAMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyWRVal > DEADBANDHIGH || joyWRVal < DEADBANDLOW)
  {
    joyWRMapped = map(joyWRVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sWristRot += joyWRMapped;
  }

  //Mapping analog joystick value to servo  signal range
  joyGripperMapped = map(joyGripperVal, 0, 1023, GRIP_MIN, GRIP_MAX);
  sGrip = joyGripperMapped;//set servo position variable to the mapped value from the knob



  sBase = min(max(g_sBase, BASE_MIN), BASE_MAX);    
  sIKY = min(max(g_sIKY, IK_MIN_Y), IK_MAX_Y);    
  sIKZ = min(max(g_sIKZ, IK_MIN_Z), IK_MAX_Z);
  sIKGA = min(max((g_sIKGA ), IK_MIN_GA), IK_MAX_GA); 
  sWristRot = min(max(g_sWristRot, WROT_MIN), WROT_MAX);

  fChanged = (sBase != g_sBase) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  


    g_bIKStatus = doArmIK(false, sBase, sIKY, sIKZ, sIKGA);

  return fChanged;

  
  
  
  
  
  
  
  
  
}


//===================================================================================================
// ProcessUserInputCylindrical90: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessAnalogInputCylindrical90() {
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
  //int   sIKX;                  // Current X value in mm
  int   sIKY;                  //
  int   sIKZ;
  int   sIKGA;    

  // Will try combination of the other two modes.  Will see if I need to do the Limits on the IK values
  // or simply use the information from the Warning/Error from last call to the IK function...
  sBase = g_sBase;
  sIKY = g_sIKY;
  sIKZ = g_sIKZ;
  sIKGA = g_sIKGA;
  
  //read analog values from analog sensors
  joyXVal = analogRead(ANALOGX);
  joyYVal = analogRead(ANALOGY);
  joyZVal = analogRead(ANALOGZ);
  joyGAVal = analogRead(ANALOGGA);
  joyWRVal = analogRead(ANALOGWR);
  joyGripperVal = analogRead(ANALOGGRIP);
  delay(5);


  


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
  {
    joyXMapped = map(joyXVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sBase += joyXMapped;
  }


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
  {
    joyYMapped = map(joyYVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKY -= joyYMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
  {
    joyZMapped = map(joyZVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKZ -= joyZMapped;
  }


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
  {
    joyGAMapped = map(joyGAVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sIKGA -= joyGAMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyWRVal > DEADBANDHIGH || joyWRVal < DEADBANDLOW)
  {
    joyWRMapped = map(joyWRVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sWristRot += joyWRMapped;
  }

  //Mapping analog joystick value to servo  signal range
  joyGripperMapped = map(joyGripperVal, 0, 1023, GRIP_MIN, GRIP_MAX);
  sGrip = joyGripperMapped;//set servo position variable to the mapped value from the knob



  sBase = min(max(g_sBase, BASE_MIN), BASE_MAX);    
  sIKY = min(max(g_sIKY, IK_MIN_Y_90), IK_MAX_Y_90);    
  sIKZ = min(max(g_sIKZ, IK_MIN_Z_90), IK_MAX_Z_90);
  sIKGA = min(max((g_sIKGA ), IK_MIN_GA), IK_MAX_GA); 
  sWristRot = min(max(g_sWristRot, WROT_MIN), WROT_MAX);

  fChanged = (sBase != g_sBase) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  


    g_bIKStatus = doArmIK(false, sBase, sIKY, sIKZ, sIKGA);

  return fChanged;

}

//===================================================================================================
// ProcessUserInputBackHoe: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessAnalogInputBackHoe() {
  
  
  
  
  boolean fChanged = false;
  

  // Will try combination of the other two modes.  Will see if I need to do the Limits on the IK values
  // or simply use the information from the Warning/Error from last call to the IK function...
  sBase = g_sBase;
  sShoulder = g_sShoulder;
  sElbow = g_sElbow;
  sWrist = g_sWrist;
  sGrip = g_sGrip;
  sWristRot = g_sWristRot;

  
  //read analog values from analog sensors
  joyXVal = analogRead(ANALOGX);
  joyYVal = analogRead(ANALOGY);
  joyZVal = analogRead(ANALOGZ);
  joyGAVal = analogRead(ANALOGGA);
  joyWRVal = analogRead(ANALOGWR);
  joyGripperVal = analogRead(ANALOGGRIP);
  delay(5);


  


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
  {
    joyXMapped = map(joyXVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sBase += joyXMapped;
  }


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
  {
    joyYMapped = map(joyYVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sShoulder -= joyYMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
  {
    joyZMapped = map(joyZVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sElbow += joyZMapped;
  }


  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
  {
    joyGAMapped = map(joyGAVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sWrist -= joyGAMapped;
  }

  //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
  if(joyWRVal > DEADBANDHIGH || joyWRVal < DEADBANDLOW)
  {
    joyWRMapped = map(joyWRVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
    g_sWristRot += joyWRMapped;
  }

  //Mapping analog joystick value to servo  signal range
  joyGripperMapped = map(joyGripperVal, 0, 1023, GRIP_MIN, GRIP_MAX);
  sGrip = joyGripperMapped;//set servo position variable to the mapped value from the knob



  sBase = min(max(g_sBase, BASE_MIN), BASE_MAX);    
  sShoulder = min(max(g_sShoulder, SHOULDER_MIN), SHOULDER_MAX);    
  sElbow = min(max(g_sElbow, ELBOW_MIN), ELBOW_MAX);
  sWrist = min(max((g_sWrist ), WRIST_MIN), WRIST_MAX); 
  sWristRot = min(max(g_sWristRot, WROT_MIN), WROT_MAX);

  fChanged = (sBase != g_sBase) || (sShoulder != g_sShoulder) || (sElbow != g_sElbow) || (sWrist != g_sWrist) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  




  return fChanged;
  
}

void ReportAnalog(unsigned char command, unsigned int value){
  unsigned char AH;
  unsigned char AL;
  AH = ((value & 0xFF00) >> 8);
  AL = (value & 0x00FF);
  Serial.write(0xff);
  Serial.write(command);
  Serial.write(AH);
  Serial.write(AL);
  Serial.write((unsigned char)(255 - (command+AH+AL)%256));
}


void IDPacket()  {
  Serial.write(0xFF);
  Serial.write((unsigned char) ARMID);
  Serial.write((unsigned char) g_bIKMode);
  Serial.write((unsigned char) 0);
  Serial.write((unsigned char)(255 - (ARMID+g_bIKMode+0)%256));
}



void DigitalOutputs(){
         // First bit = D1, 2nd bit = D2, etc. 
        int i;
        for(i=0;i<7;i++){
        unsigned char button = (armlink.buttons>>i)&0x01;
        if(button > 0){
          // button pressed, go high on a pin
          DDRB |= 0x01<<(i+1);
          PORTB |= 0x01<<(i+1);
        }
        else{
          DDRB &= 0xff - (0x01<<(i+1));
          PORTB &= 0xff - (0x01<<(i+1));
        } 
      } 
}



//=============================================================================
//=============================================================================
#endif
