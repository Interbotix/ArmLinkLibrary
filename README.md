Arm Link Library and Firmware
====================


This repo houses the Arduino Arm Link Library and the Arm Link Firmware examples for ArbotiX Robocontroller/ InterbotiX Robot Arms.

These files are compatible with all three InterbotiX Robot Arms.

PhantomX Pincher Robot Arm Kit
http://learn.trossenrobotics.com/interbotix/robot-arms/pincher-arm

PhantomX Reactor Robot Arm Kit
http://learn.trossenrobotics.com/interbotix/robot-arms/reactor-arm

WidowX Robot Arm Kit 
http://learn.trossenrobotics.com/interbotix/robot-arms/widowx-arm





<h2>Installation</h2>
These library/firmware files are avaialble as part of the 'arbotix' hardware/libraries installation. If you have loaded the latest version of the 'arbotix' repo, you will have these files installed already. For more information see the 'ArbotiX Getting Started Guide'
http://learn.trossenrobotics.com/arbotix/7-arbotix-quick-start-guide


For manual installation, copy the 'ArmLink' folder into your Arduino 'Libraries' folder. Restart the Arduino environment. Firmware examples can be found under


File->Examples->ArmLink


<h2>Arm Link Library</h2>
The Arm Link library provides structure for communicating with an InterbotiX Arm running the ArmLink Firmware over a serial port. 


The Arm Link Library is only required for communicating with the arm over a serial port. The Arm Link Library does not handle any Inverse kinematics or other Arm functions.


For more information on the Arm Link protocol and library, see this article
http://learn.trossenrobotics.com/36-demo-code/137-interbotix-arm-link-software.html



<h3>Arm Link Firmware</h3>
The Arm Link Firmware examples are  Arduino Firmware files to be loaded onto the ArbotiX Robocontroller.

As this firmware is compatible with multiple arms, you will need to select your arm in the code. In the main .ino file under the main comments, find the following code block

```
//=============================================================================
// Define Options
//=============================================================================

#define NOPE
//#define PINCHER
//#define REACTOR
//#define WIDOWX
```

Comment out '#define NOPE' and uncomment the arm you are using. For example, if you are using the Pincher Arm, your code should look like

```
//=============================================================================
// Define Options
//=============================================================================

//#define NOPE
#define PINCHER
//#define REACTOR
//#define WIDOWX
```




<h4>ArmLinkSerial</h4>
This firmware will repsond to Arm Link style serial commands. This included serial commands from the Arm Link PC Software. 

See this article for more information
http://learn.trossenrobotics.com/36-demo-code/137-interbotix-arm-link-software.html


<h4>ArmLinkPlayback</h4>

Note: This firmware does not use the ArmLink Library. It is offered in this repo for workflow purposes. See this article for more information
http://learn.trossenrobotics.com/36-demo-code/137-interbotix-arm-link-software.html


<h4>ArmLinkAnalog</h4>

This example will allow you to contorl the Robot Arm via analog controls. 

Note: This firmware does not use the ArmLink Library. It is offered in this repo for workflow purposes. See this article for more information
http://learn.trossenrobotics.com/36-demo-code/137-interbotix-arm-link-software.html





This firmware is used to control the arm via serial packet, which breaks out absolute X, Y, Z Axis control, Wrist angle, Wrist rotation, Gripper control, as well as speed control. In addition, it also has the ability to control digital outputs on the arbotix, read analog inputs, and switch between several control/IK modes. More information to come.




