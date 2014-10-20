extern void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause);

void SequenceLoop(){
  delay(500);
  Serial.println("Sequencing Mode Active."); 
  Serial.println("Send '1' or press Button 1 to pause and return to menu.");
  loopbreak = LOW;  
  do
  {

    
      IKSequencingControl(-200, 235, 210, 0, 512, 512, 1000, 0);
    
    for(int i = -200; i < 200 ; i++)
    {
      
      IKSequencingControl(i, 235, 210, 0, 512, 512, 2, 0);
    }
    
    
//    
//    //###########################################################//
//    // SEQUENCE 2
//    //###########################################################//
//    Serial.println("Moving to Sequence Position 2");  
//    IKSequencingControl(100, 235, 210, 0, 512, 512, 10, 0);
//    //###########################################################//
//
//
//    //###########################################################//
//    // SEQUENCE 2
//    //###########################################################//
//    Serial.println("Moving to Sequence Position 2");  
//    IKSequencingControl(-100, 235, 210, 0, 512, 512, 500, 100);
//    //###########################################################//
//    
//    
//    //###########################################################//
//    // SEQUENCE 1
//    //###########################################################// 
//    //IKSequencingControl(X-axis, Y-Axis, Z-axis, Wrist Angle, Gripper, Interpolation, Delay)
//    Serial.println("Moving to Sequence Position 1");   
//    IKSequencingControl(0, 235, 210, 0, 512, 512, 500, 100);
//    //###########################################################//
//
//
//


  } 
  while((Serial.available() == 0) && (loopbreak == LOW));  
  Serial.read(); // Read & discard the character that got us out of the loop.
  delay(100);
  Serial.println("Pausing Sequencing Mode."); 
  delay(500);
 // MenuOptions();
}

