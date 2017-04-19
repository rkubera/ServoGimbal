/********************************************************
 * ServoGimbal Font-End, Version 0.1
 * by Radek Kubera
 * based on Arduino PID Tuning Front-End,  Version 0.3
 * by Brett Beauregard
 ********************************************************/
/********************************************************
 * Arduino PID Tuning Front-End,  Version 0.3
 * by Brett Beauregard
 * License: Creative-Commons Attribution Share-Alike
 * April 2011
 *
 * This application is designed to interface with an
 * arduino running the PID Library.  From this Control
 * Panel you can observe & adjust PID performance in 
 * real time
 *
 * The ControlP5 library is required to run this sketch.
 * files and install instructions can be found at
 * http://www.sojamo.de/libraries/controlP5/
 * 
 ********************************************************/

import java.nio.ByteBuffer;
import processing.serial.*;
import controlP5.*;

/***********************************************
 * User spcification section
 **********************************************/
int windowWidth = 1000;      // set the size of the 
int windowHeight = 720;     // form

float InScaleMin = -90;       // set the Y-Axis Min
float InScaleMax = 90;    // and Max for both
float OutScaleMin = -90;      // the top and 
float OutScaleMax = 90;    // bottom trends

int windowSpan = 30000;    // number of mS into the past you want to display
int refreshRate = 100;      // how often you want the graph to be reDrawn;

float displayFactor = 1; //display Time as Milliseconds
//float displayFactor = 1000; //display Time as Seconds
//float displayFactor = 60000; //display Time as Minutes

String outputFileName = ""; // if you'd like to output data to 
// a file, specify the path here

/***********************************************
 * end user spec
 **********************************************/

int nextRefresh;
int arrayLength = windowSpan / refreshRate+1;
int[] InputData = new int[arrayLength];     //we might not need them this big, but
int[] SetpointData = new int[arrayLength];  // this is worst case
int[] OutputData = new int[arrayLength];

float inputTop = 25;
float inputHeight = (windowHeight-70)*1/2;
float outputTop = inputHeight+50;
float outputHeight = (windowHeight-70)*1/2;

float ioLeft = 160, ioWidth = windowWidth-ioLeft-50;
float ioRight = ioLeft+ioWidth;
float pointWidth = (ioWidth)/float(arrayLength-1);

int ConnectPanelPos = 5;
int ConnectPanelHeight = 125;
int GimbalPanelPos = 450;
int GimbalPanelHeight = 265;
//int PIDPanelPos = 300;
int PIDPanelPos = 135;
int PIDPanelHeight = 310;

int vertCount = 10;

int nPoints = 0;

float Input, Setpoint, Output;

boolean madeContact =false;
boolean justSent = true;
boolean justChangedRollPitch = true;
boolean justChangedConfig = true;

boolean connected = false;

Serial myPort;

ControlP5 controlP5;
controlP5.Button DRButton, PIDButton, GyroButton, ConnectButton, PWMButton, MavRollButton, MavPitchButton ;
controlP5.Textlabel InLabel, OutLabel, PLabel, SPLabel, SetPointLabel, PIDLabel, ConnectedLabel, ILabel, DLabel,DRLabel, DRCurrent, InputLabel, OutputLabel, PWMLabel, MavRollLabel, MavPitchLabel;
controlP5.Textfield InField, OutField, PField, IField, DField;
ListBox PortList, MavRollList, MavPitchList;

PrintWriter output;
PFont AxisFont, TitleFont; 

void setup() {
  frameRate(30);
  surface.setSize(windowWidth , windowHeight);

  controlP5 = new ControlP5(this);                                  // * Initialize the various
  
  //PID Section
  //PIDButton = controlP5.addButton("Toggle_PID",0.0,10,PIDPanelPos+30,60,20);
  PIDButton = controlP5.addButton("Toggle_PID").setPosition(10,PIDPanelPos+30).setSize(60,20);
  PIDLabel = controlP5.addTextlabel("PID","ROLL",80,  PIDPanelPos+35);
  InLabel=controlP5.addTextlabel("In","1",80,PIDPanelPos+65);
  OutLabel=controlP5.addTextlabel("Out","2",80,PIDPanelPos+85);
  //SPField= controlP5.addTextfield("Setpoint",10,PIDPanelPos+105,60,20);
  SPLabel=controlP5.addTextlabel("SP","3",80,PIDPanelPos+105);
  SetPointLabel=controlP5.addTextlabel("SetPoint","Setpoint value:",10,PIDPanelPos+105);
  
  PField = controlP5.addTextfield("Kp (Proportional)",10,PIDPanelPos+125,60,20); 
  PLabel=controlP5.addTextlabel("P","4",80,PIDPanelPos+130);
  IField = controlP5.addTextfield("Ki (Integral)",10,PIDPanelPos+165,60,20);
  ILabel=controlP5.addTextlabel("I","5",80,PIDPanelPos+170);
  DField = controlP5.addTextfield("Kd (Derivative)",10,PIDPanelPos+205,60,20);
  DLabel=controlP5.addTextlabel("D","6",80,PIDPanelPos+210);
  
  DRButton = controlP5.addButton("Toggle_DR").setPosition(10,PIDPanelPos+245).setSize(60,20);
  DRLabel = controlP5.addTextlabel("DR","Direct",10,PIDPanelPos+270);
  DRCurrent = controlP5.addTextlabel("DRCurrent","Direct",80,PIDPanelPos+250);

  controlP5.addButton("Send_To_Controller").setPosition(10,PIDPanelPos+285).setSize((int)ioLeft-20,20);
  InputLabel=controlP5.addTextlabel("Input","Input value:",10,PIDPanelPos+65);
  OutputLabel=controlP5.addTextlabel("Output","Output value:",10,PIDPanelPos+85);
  
  //Gimbal section 
  GyroButton = controlP5.addButton("Detect_Gyro_Orientation").setPosition(10,GimbalPanelPos+30).setSize((int)ioLeft-20,20);
  PWMButton = controlP5.addButton("PWM_Freq").setPosition(10,GimbalPanelPos+60).setSize(60,20);
  PWMLabel = controlP5.addTextlabel("PWM_FreqLabel","50Hz",80,GimbalPanelPos+65);
  
  MavRollButton = controlP5.addButton("Mav_Roll_RC_Channel").setPosition(10,GimbalPanelPos+90).setSize(110,20);
  MavRollLabel = controlP5.addTextlabel("Mavlink_Roll_RC_ChannelLabel","1",130,GimbalPanelPos+95);

  MavPitchButton = controlP5.addButton("Mav_Pitch_RC_Channel").setPosition(10,GimbalPanelPos+120).setSize(110,20);
  MavPitchLabel = controlP5.addTextlabel("Mavlink_Pitch_RC_ChannelLabel","1",130,GimbalPanelPos+125); 
  
  MavRollLabel.hide();
  MavPitchLabel.hide();
  MavRollButton.hide();
  MavPitchButton.hide();
        
  //Serial port section
  PortList = controlP5.addListBox("Serial_Port").setPosition(10,ConnectPanelPos+30).setSize((int)ioLeft-20,70).setItemHeight(20).setBarHeight(20);
  ConnectButton = controlP5.addButton("Connect").setPosition(10,ConnectPanelPos+100).setSize(60,20);
  ConnectedLabel=controlP5.addTextlabel("Connected","Disconnected",80,ConnectPanelPos+105); 
  for (int i=0; i<Serial.list().length;i++) {
   PortList.addItem(Serial.list()[i],Serial.list()[i]);
  }
 
  AxisFont = loadFont("axis.vlw");
  TitleFont = loadFont("Titles.vlw");
 
  nextRefresh=millis();
  if (outputFileName!="") output = createWriter(outputFileName);
}

void draw() {
  background(200);
  drawGraph();
  drawButtonArea();
}

void drawGraph() {
  //draw Base, gridlines
  stroke(0);
  fill(230);
  rect(ioLeft, inputTop,ioWidth-1 , inputHeight);
  rect(ioLeft, outputTop, ioWidth-1, outputHeight);
  stroke(210);

  //Section Titles
  textFont(TitleFont);
  fill(255);
  
  text("PID Input / Setpoint",(int)ioLeft+10,(int)inputTop-5);
  text("PID Output",(int)ioLeft+10,(int)outputTop-5);

  //GridLines and Titles
  textFont(AxisFont);
  
  //horizontal grid lines
  int interval = (int)inputHeight/5;
  for(int i=0;i<6;i++) {
    if(i>0&&i<5) line(ioLeft+1,inputTop+i*interval,ioRight-2,inputTop+i*interval);
    text(str((InScaleMax-InScaleMin)/5*(float)(5-i)+InScaleMin),ioRight+5,inputTop+i*interval+4);

  }
  interval = (int)outputHeight/5;
  for(int i=0;i<6;i++) {
    if(i>0&&i<5) line(ioLeft+1,outputTop+i*interval,ioRight-2,outputTop+i*interval);
    text(str((OutScaleMax-OutScaleMin)/5*(float)(5-i)+OutScaleMin),ioRight+5,outputTop+i*interval+4);
  }

  //vertical grid lines and TimeStamps
  int elapsedTime = millis();
  interval = (int)ioWidth/vertCount;
  int shift = elapsedTime*(int)ioWidth / windowSpan;
  shift %=interval;

  int iTimeInterval = windowSpan/vertCount;
  float firstDisplay = (float)(iTimeInterval*(elapsedTime/iTimeInterval))/displayFactor;
  float timeInterval = (float)(iTimeInterval)/displayFactor;
  for(int i=0;i<vertCount;i++) {
    int x = (int)ioRight-shift-2-i*interval;

    line(x,inputTop+1,x,inputTop+inputHeight-1);
    line(x,outputTop+1,x,outputTop+outputHeight-1);    

    float t = firstDisplay-(float)i*timeInterval;
    if(t>=0)  text(str(t),x,outputTop+outputHeight+10);
  }

  // add the latest data to the data Arrays.  the values need
  // to be massaged to get them to graph correctly.  they 
  // need to be scaled to fit where they're going, and 
  // because 0, 0 is the top left, we need to flip the values.
  // this is easier than having the user stand on their head
  // to read the graph.
  
  if (madeContact) {
     ConnectedLabel.setValue("Connected");
  }
  if(!madeContact && connected) {
    myPort.write("p");
  }
  else if(millis() > nextRefresh && madeContact) {
    nextRefresh += refreshRate;

    for(int i=nPoints-1;i>0;i--) {
      InputData[i]=InputData[i-1];
      SetpointData[i]=SetpointData[i-1];
      OutputData[i]=OutputData[i-1];
    }
    if (nPoints < arrayLength) nPoints++;

    InputData[0] = int(inputHeight)-int(inputHeight*(Input-InScaleMin)/(InScaleMax-InScaleMin));
    SetpointData[0] =int( inputHeight)-int(inputHeight*(Setpoint-InScaleMin)/(InScaleMax-InScaleMin));
    OutputData[0] = int(outputHeight)-int(outputHeight*(Output-OutScaleMin)/(OutScaleMax-OutScaleMin));
  }
  //draw lines for the input, setpoint, and output
  strokeWeight(2);
  for(int i=0; i<nPoints-2; i++) {
    int X1 = int(ioRight-2-float(i)*pointWidth);
    int X2 = int(ioRight-2-float(i+1)*pointWidth);
    boolean y1Above, y1Below, y2Above, y2Below;

    //DRAW THE INPUT
    boolean drawLine=true;
    stroke(255,0,0);
    int Y1 = InputData[i];
    int Y2 = InputData[i+1];

    y1Above = (Y1>inputHeight);                     // if both points are outside 
    y1Below = (Y1<0);                               // the min or max, don't draw the 
    y2Above = (Y2>inputHeight);                     // line.  if only one point is 
    y2Below = (Y2<0);                               // outside constrain it to the limit, 
    if(y1Above) {                                   // and leave the other one untouched.
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)inputHeight;                      //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)inputHeight;                   //
    }                                               //
    else if(y1Below) {                              //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)inputHeight;                      //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else {                                          //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)inputHeight;       //
    }                                               //

    if(drawLine) {
      line(X1,Y1+inputTop, X2, Y2+inputTop);
    }

    //DRAW THE SETPOINT
    drawLine=true;
    stroke(0,255,0);
    Y1 = SetpointData[i];
    Y2 = SetpointData[i+1];

    y1Above = (Y1>(int)inputHeight);                // if both points are outside 
    y1Below = (Y1<0);                               // the min or max, don't draw the 
    y2Above = (Y2>(int)inputHeight);                // line.  if only one point is 
    y2Below = (Y2<0);                               // outside constrain it to the limit, 
    if(y1Above) {                                   // and leave the other one untouched.
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)(inputHeight);                    //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)(inputHeight);                 //
    }                                               //
    else if(y1Below) {                              //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)(inputHeight);                    //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else {                                          //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)(inputHeight);     //
    }                                               //

    if(drawLine) {
      line(X1, Y1+inputTop, X2, Y2+inputTop);
    }

    //DRAW THE OUTPUT
    drawLine=true;
    stroke(0,0,255);
    Y1 = OutputData[i];
    Y2 = OutputData[i+1];

    y1Above = (Y1>outputHeight);                   // if both points are outside 
    y1Below = (Y1<0);                              // the min or max, don't draw the 
    y2Above = (Y2>outputHeight);                   // line.  if only one point is 
    y2Below = (Y2<0);                              // outside constrain it to the limit, 
    if(y1Above) {                                  // and leave the other one untouched.
      if(y2Above) drawLine=false;                  //
      else if(y2Below) {                           //
        Y1 = (int)outputHeight;                    //
        Y2 = 0;                                    //
      }                                            //
      else Y1 = (int)outputHeight;                 //
    }                                              //
    else if(y1Below) {                             //
      if(y2Below) drawLine=false;                  //
      else if(y2Above) {                           //
        Y1 = 0;                                    //
        Y2 = (int)outputHeight;                    //
      }                                            //  
      else Y1 = 0;                                 //
    }                                              //
    else {                                         //
      if(y2Below) Y2 = 0;                          //
      else if(y2Above) Y2 = (int)outputHeight;     //
    }                                              //

    if(drawLine) {
      line(X1, outputTop + Y1, X2, outputTop + Y2);
    }
  }
  strokeWeight(1);
}

void Connect() {
  if (connected==false) {
    myPort = new Serial(this, PortList.getValueLabel().getText(), 115200);      //   Communication with
    myPort.bufferUntil(10);  
    //ConnectedLabel.setValue("Connected");
    connected = true;
  }
}

void drawButtonArea() {
  stroke(0);
  fill(100);
  rect(0, 0, ioLeft, windowHeight);
  
  //Serial port Section
  stroke(0);
  fill(70);
  rect(5, ConnectPanelPos, ioLeft-10, ConnectPanelHeight);
  textFont(TitleFont);
  fill(255);
  text("SERIAL PORT",10,ConnectPanelPos+20);

//Gimbal Section
  stroke(0);
  fill(70);
  rect(5, GimbalPanelPos, ioLeft-10, GimbalPanelHeight);
  textFont(TitleFont);
  fill(255);
  text("GIMBAL SETTINGS",10,GimbalPanelPos+20);
  
//PID Section
  stroke(0);
  fill(70);
  rect(5, PIDPanelPos, ioLeft-10, PIDPanelHeight);
  textFont(TitleFont);
  fill(255);
  text("PID SETTINGS",10,PIDPanelPos+20);
}

void PWM_Freq() {
  if(PWMLabel.getValueLabel().getText().equals("50Hz")) {
    PWMLabel.setValue("333Hz");
  }
  else {
    PWMLabel.setValue("50Hz");
  }
  SendConfig();
  justChangedConfig = true;
}

void Detect_Gyro_Orientation() {
  myPort.write("b");
}

void Toggle_PID() {
  if(PIDLabel.getValueLabel().getText()=="PITCH") {
    PIDLabel.setValue("ROLL");
  }
  else {
    PIDLabel.setValue("PITCH");   
  }
  justChangedRollPitch = true;
}

void Toggle_DR() {
  if(DRLabel.getValueLabel().getText()=="Direct") {
    DRLabel.setValue("Reverse");
  }
  else {
    DRLabel.setValue("Direct");   
  }
}

void Mav_Roll_RC_Channel() {
  int val = int(MavRollLabel.getValueLabel().getText() );
  val++;
  if (val>16) val = 1;
  MavRollLabel.setText (str(val));
  SendConfig();
  justChangedConfig = true;
}

void Mav_Pitch_RC_Channel() {
  int val = int(MavPitchLabel.getValueLabel().getText() );
  val++;
  if (val>16) val = 1;
  MavPitchLabel.setText (str(val));
  SendConfig();
  justChangedConfig = true;
}

void SendConfig() {
  myPort.write("m");
  if(PWMLabel.getValueLabel().getText().equals("50Hz")) {
    myPort.write((byte)1);
  }
  else {
    myPort.write((byte)2);
  }
  myPort.write((byte) int(MavRollLabel.getValueLabel().getText()));
  myPort.write((byte) int(MavPitchLabel.getValueLabel().getText()));
}


// Sending Floating point values to the arduino
// is a huge pain.  if anyone knows an easier
// way please let know.  the way I'm doing it:
// - Take the 6 floats we need to send and
//   put them in a 6 member float array.
// - using the java ByteBuffer class, convert
//   that array to a 24 member byte array
// - send those bytes to the arduino
void Send_To_Controller() {
  float[] toSend = new float[6];
  //toSend[0] = float(SPField.getText());
  toSend[0] = 0;
  toSend[1] = 0;
  toSend[2] = 0;
  toSend[3] = float(PField.getText());
  toSend[4] = float(IField.getText());
  toSend[5] = float(DField.getText());
  Byte a = (byte)1;
  Byte d = (DRLabel.getValueLabel().getText()=="Direct")?(byte)0:(byte)1;
  Byte c = (PIDLabel.getValueLabel().getText()=="ROLL")?(byte)0:(byte)1;
  myPort.write(a);
  myPort.write(d);
  myPort.write(c);
  myPort.write(floatArrayToByteArray(toSend));
  justSent=true;
} 

byte[] floatArrayToByteArray(float[] input) {
  int len = 4*input.length;
  byte[] b = new byte[4];
  byte[] out = new byte[len];
  ByteBuffer buf = ByteBuffer.wrap(b);
  for(int i=0;i<input.length;i++) {
    buf.position(0);
    buf.putFloat(input[i]);
    for(int j=0;j<4;j++) out[j+i*4]=b[3-j];
  }
  return out;
}

//take the string the arduino sends us and parse it
void serialEvent(Serial myPort) {
  String read = myPort.readStringUntil(10);
//  println(read);
  if(outputFileName!="") output.print(str(millis())+ " "+read);
  String[] s = split(trim(read), " ");
  if (trim(s[0]).equals("CONFIG")) {
    if(justChangedConfig) {
      justChangedConfig = false;
      PWMLabel.setText(trim(s[1]+"Hz"));
      MavRollLabel.setText (trim(s[2]));
      MavPitchLabel.setText (trim(s[3]));
      if (trim(s[2]).equals("0")) {
        MavRollLabel.hide();
        MavPitchLabel.hide();
        MavRollButton.hide();
        MavPitchButton.hide();
      }
      else {
        MavRollLabel.setVisible(true);
        MavPitchLabel.setVisible(true);
        MavRollButton.setVisible(true);
        MavPitchButton.setVisible(true);
      }
      if(!madeContact) madeContact=true;
    }
  }
    
  if (trim(s[0]).equals("PID") && s.length ==10 && trim(s[9]).equals(PIDLabel.getValueLabel().getText())) {
    Setpoint = float(s[1]);           // * pull the information
    Input = float(s[2]);              //   we need out of the
    Output = float(s[3]);             //   string and put it
    SPLabel.setValue(s[1]);           //   where it's needed
    InLabel.setValue(s[2]);           //
    OutLabel.setValue(trim(s[3]));    //
    PLabel.setValue(trim(s[4]));      //
    ILabel.setValue(trim(s[5]));      //
    DLabel.setValue(trim(s[6]));      //
    DRCurrent.setValue(trim(s[8]));
    if(justSent || justChangedRollPitch) {  //
      justSent=false;                 //
      justChangedRollPitch = false;
      //SPField.setText(trim(s[1]));    //   
      PField.setText(trim(s[4]));     //
      IField.setText(trim(s[5]));     //
      DField.setText(trim(s[6]));     //
      DRCurrent.setValue(trim(s[8]));         //
      if(!madeContact) madeContact=true;
    }
  }
}