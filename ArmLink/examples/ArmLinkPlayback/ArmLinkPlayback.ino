/***********************************************************************************
 *  }--\     RInterbotiX Robotic Arm           /--{
 *      |       Analog IK Control Code        |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will move the arm to an X/Y/Z coordinate based on the inputs
 *  from the analog inputs (joysticks and knob). This sketch can also be used to play
 *  back a pre-programmed sequence.
 *
 *  Snapper Arm Getting Started Guide
 *   http://learn.robotgeek.com/getting-started/33-robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide.html
 *  Using the IK Firmware
 *    http://learn.robotgeek.com/demo-code/demo-code/154-robotgeek-snapper-joystick-inverse-kinematics-demo.html
 *
 *
 *  WIRING
 *    Servos
 *      Digital I/O 3 - Base Rotation - Robot Geek Servo 
 *      Digital I/O 5 - Shoulder Joint - Robot Geek Servo 
 *      Digital I/O 6 - Elbow Joint - Robot Geek Servo 
 *      Digital I/O 9 - Wrist Joint - Robot Geek Servo 
 *      Digital I/O 10 - Gripper Servo - 9g Servo 
 *
 *    Analog Inputs
 *      Analog 0 - Joystick (Horizontal)
 *      Analog 1 - Joystick (Vertical)
 *      Analog 2 - Joystick (Vertical)
 *      Analog 3 - Joystick (Vertical)
 *      Analog 4 - Rotation Knob 
 *      
 *    Digital Inputs
 *      Digital 2 - Button 1
 *      Digital 4 - Button 2
 *
 *  
 *    Use an external power supply and set both PWM jumpers to 'VIN'
 *
 *  CONTROL
 *    Turn the 
 *
 *
 *  NOTES
 *    ANALOG INPUT MAPPING
 *      This code uses a combination of direct and incremental code for converting 
 *      analog inputs into servo positions
 *    
 *      Direct/Absolute
 *        Absolute positioning is used for the knobs controlling the base and gripper servo.
 *        This means that the value of the knob is mapped directly to the corresponding servo
 *        value. This method is ideal for sensors that stay at static positions such as
 *        knobs and sliders. 
 *    
 *      Incremental
 *        Incremental code is used for the joysticks controlling the shoulder, elbow and
 *        gripper servo. Each joystick value is mapped to a small realtiveley small positive
 *        or negative value. This value is then added to the currrent position of the servo.
 *        The action of slowly moving the joystick away from its center position can slowly 
 *        move each joint of the robot. When the joystick is centered, no movement is made
 *     
 *      The choice for using Direct/Incremental mapping for each joint was made based
 *      on usability, however the code can be modified so that any joint can use
 *      either direct or incremental mapping
 *
 *    SERVO POSITIONS
 *      The servos' positions will be tracked in microseconds, and written to the servos
 *      using .writeMicroseconds()
 *        http://arduino.cc/en/Reference/ServoWriteMicroseconds
 *      For RobotGeek servos, 600ms corresponds to fully counter-clockwise while
 *      2400ms corresponds to fully clock-wise. 1500ms represents the servo being centered 
 *
 *      For the 9g servo, 900ms corresponds to fully counter-clockwise while
 *      2100ms corresponds to fully clock-wise. 1500ms represents the servo being centered 
 *
 *
 *  This code is a Work In Progress and is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 * Sources used:
 * https://github.com/KurtE
 * 
 * http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 * 
 * Application Note 44 - Controlling a Lynx6 Robotic Arm 
 * http://www.micromegacorp.com/appnotes.html
 * http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf
 * 
 * 
 * //  This code is a Work In Progress and is distributed in the hope that it will be useful,
 * //  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * //  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 * //  
 * //=============================================================================
 * 
 * Sources used:
 * http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 * 
 * Application Note 44 - Controlling a Lynx6 Robotic Arm 
 * http://www.micromegacorp.com/appnotes.html
 * http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf
 * 
 ***********************************************************************************/
 
 




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

#define NOPE
//#define PINCHER
//#define REACTOR
//#define WIDOWX

#define ENABLE_ANALOG FA


#define SOUND_PIN    7      // Tell system we have added speaker to IO pin 1
#define MAX_SERVO_DELTA_PERSEC 512
//#define DEBUG             // Enable Debug mode via serial

//=============================================================================
// Global Include files
//=============================================================================
//DYNAMIXEL Control libraries
#include <ax12.h>
#include <BioloidController.h>
//ArmLink library
#include <ArmLink.h>
//input control file - local
#include "InputControl.h"

// Definition of interrupt names
#include < avr/io.h >
// ISR interrupt service routine
#include < avr/interrupt.h >


//=============================================================================
// Global Objects
//=============================================================================
BioloidController bioloid = BioloidController(1000000);
ArmLink armlink = ArmLink();



// Message information
unsigned long   ulLastMsgTime;          // Keep track of when the last message arrived to see if controller off
byte            buttonsPrev;            // will use when we wish to only process a button press once



long lastDebounceTime = 0;  // the last time the output pin was changed
long debounceDelay = 50;    // the debounce time - the button needs to be held for at least this long in ms


// We need to declare the data exchange
// variable to be volatile - the value is
// read from memory.
volatile int playState = 0; // 0 = stopped 1 = playing

volatile long lastInterruptChange;


//===================================================================================================
// Setup 
//====================================================================================================
void setup() {
  
  attachArbotixInterrupt(FALLING);
  
  
  //Serial activity LED output
  pinMode(0,OUTPUT);  
  // Lets initialize the Serial Port
  Serial.begin(38400);
  delay(50);
  IDPacket();
  Serial.println("Interbotix Robot Arm Online.");

  // Next initialize the Bioloid
  bioloid.poseSize = CNT_SERVOS;

  // Read in the current positions...
  bioloid.readPose();
  delay(100);
  // Start off to put arm to sleep...
  PutArmToSleep();
  //Send ID Packet

  MSound(3, 60, 2000, 80, 2250, 100, 2500);

  //set Gripper Compliance so it doesn't tear itself apart
  ax12SetRegister(SID_GRIP, AX_CW_COMPLIANCE_SLOPE, 128);
  ax12SetRegister(SID_GRIP, AX_CCW_COMPLIANCE_SLOPE, 128);




  pinMode(BUTTON1, INPUT);     
  pinMode(BUTTON2, INPUT);     
  digitalWrite(BUTTON1,HIGH);  
  digitalWrite(BUTTON2,HIGH);  
  
        g_bIKMode = IKM_IK3D_CARTESIAN;
        MoveArmToHome(); 
  
}


//===================================================================================================
// loop: Our main Loop!
//===================================================================================================
void loop() 
{
  boolean fChanged = false;
  
  //use digitalRead to store the current state of the pushbutton in one of the 'buttonState' variables
  buttonState1 = digitalRead(BUTTON1);
  
  if (buttonState1 == LOW) 
  {     
    SequenceLoop();
  } 
  
  int inByte = Serial.read();

  switch (inByte) 
  {
    case '1':    
    SequenceLoop(); 
    break;    
  }
  
  
  if (bioloid.interpolating > 0) 
  {
    bioloid.interpolateStep();
  }
} 


void SequenceLoop()
{
  delay(500);
  Serial.println("Sequencing Mode Active."); 
  Serial.println("Send '1' or press Button 1 to pause and return to menu.");
  playState = 1;  //set playState to 1 as the sequence is now playing
  do
  {
    //IKSequencingControl(-200, 235, 210, 0, 512, 512, 2000, 1000);
    //IKSequencingControl(200, 235, 210, 0, 512, 512, 2000, 5000);
    //IKSequencingControl(-200, 235, 210, 0, 512, 512, 2000, 1000);
    
    
    g_bIKMode = IKM_CYLINDRICAL;
  
    
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    IKSequencingControl(2048 , 250 , 225 , 0 , 0 , 256 , 2000 , 1000, playState);
    //###########################################################// 
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    IKSequencingControl(1332 , 250 , 225 , 0 , 511 , 24 , 2000 , 1000, playState);
    //###########################################################// 
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    IKSequencingControl(2967 , 250 , 225 , 0 , -493 , 512 , 2000 , 1000, playState);
    //###########################################################// 
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    IKSequencingControl(304 , 143 , 258 , 0 , -5 , 53 , 2000 , 1000, playState);
    //###########################################################// 
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    IKSequencingControl(304 , 143 , 258 , 21 , -5 , 53 , 1000 , 1000, playState);
    //###########################################################// 
    
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    IKSequencingControl(1844 , 261 , 167 , 21 , -5 , 53 , 4080 , 1000, playState);
    //###########################################################// 

  } 
  while((Serial.available() == 0) && (playState == 1));  //if a serial command is received or the playState variable changes via intterupt), stop the loop
    
  Serial.read(); // Read & discard the character that got us out of the loop.
  delay(100);
  Serial.println("Pausing Sequencing Mode."); 
  delay(500);
 // MenuOptions();
}




void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable)
{
  if(enable == 1)
  {
    if(g_bIKMode == IKM_IK3D_CARTESIAN || g_bIKMode == IKM_IK3D_CARTESIAN_90)
    {
      doArmIK(true, X, Y, Z, GA); 
      
    }
    else if(g_bIKMode == IKM_CYLINDRICAL || g_bIKMode ==IKM_CYLINDRICAL_90)
    {  
    //  sBase = X;
      doArmIK(false, X, Y, Z, GA); 
      
    }
    else if(g_bIKMode == IKM_BACKHOE)
    {
      sBase = X;
      sShoulder = Y;
      sElbow = Z;
      sWrist = GA;
      
    }
    
    
    
    sWristRot = WR;
    sGrip = grip;
  
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, interpolate, true);  
    delay(pause);
  }
}



void attachArbotixInterrupt(int interruptType)
{
  
  // Global Enable INT2 interrupt - pin 2 on the arbotiX
  EIMSK |= ( 1 << INT2);
  
  
  // Signal change triggers interrupt
  if(interruptType == LOW)
  {
    EICRA |= ( 0 << ISC20);
    EICRA |= ( 0 << ISC21);
  }
  else if(interruptType == CHANGE)
  {
    EICRA |= ( 1 << ISC20);
    EICRA |= ( 0 << ISC21);
  }
  else if(interruptType == FALLING)
  {
    EICRA |= ( 1 << ISC20);
    EICRA |= ( 0 << ISC21);
  }
  else if(interruptType == RISING)
  {
    EICRA |= ( 1 << ISC20);
    EICRA |= ( 1 << ISC21);
  }
    
}

// Install the interrupt routine.
ISR(INT2_vect) {
  
  if( millis() - lastInterruptChange >100)
  {
      
    // check the value again - since it takes some time to
    // activate the interrupt routine, we get a clear signal.
    if(playState == 1)
    {
      playState = !playState;
    }
    lastInterruptChange = millis();
    
  }
  
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




