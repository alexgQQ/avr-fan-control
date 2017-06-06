import controlP5.*;
import processing.serial.*; 

ControlP5 gui;
Slider slider1, slider2;
Serial myPort;                      // The serial port
PFont myFont;                       // The display font
String inString = "XXX";            // Input string from serial port

char[] cpuSpeed = new char[2];      // The prvious and current Fan#1 speed
char[] gpuSpeed = new char[2];      // The prvious and current Fan#2 speed
int cpuRX = 0;                      // received measurment for fan #1
int gpuRX = 0;                      // received measurment for fan #2
 
void setup() { 
  size(600,100); 

  myFont = createFont("pixelmix.ttf", 8, false);  // Load font, font file should be in data directory

  textFont(myFont);

  serialInit(38400);          // MUST BE SAME BAUDRATE AS CODE ON ATMEGA328

  guiInit();

  delay(1000);                // Startup delay
} 
 
void draw() { 

  background(0);

  cpuSpeed[1] = (char)slider1.getValue();
  gpuSpeed[1] = (char)slider2.getValue();

  senseChange();

  fill(255);

  readRpm();
}


/*****************************************************
* NAME:         readRpm
*
* DESCRIPTION:  Prints RPM values next to slider bars
*
*
* INPUTS:       None
* 
* OUTPUTS:      None
*
* NOTES:        
*
*/  
void readRpm()
{
  text("SPEED " + str(cpuRX*60) + " RPM", 400,  34);
  text("SPEED " + str(gpuRX*60) + " RPM", 400,  64);
}



/*****************************************************
* NAME:         senseChange
*
* DESCRIPTION:  Detects if a speed change has been initiated by a slider bar. 
*               Sends updated speed immediatley. 
*
*
* INPUTS:       None
* 
* OUTPUTS:      None
*
* NOTES:        
*
*/  
void senseChange()
{
  if(cpuSpeed[0] != cpuSpeed[1])
  {
    cpuSpeed[0] = cpuSpeed[1];
    sendSpeed((byte)0x01, (byte)cpuSpeed[0]);
  }
  else if(gpuSpeed[0] != gpuSpeed[1])
  {
    gpuSpeed[0] = gpuSpeed[1];
    sendSpeed((byte)0x02, (byte)gpuSpeed[0]);
  }
    
}


/*****************************************************
* NAME:         guiInit
*
* DESCRIPTION:  Initiallizes graphical objects for a basic slider bar menu.  
*
*
* INPUTS:       None
* 
* OUTPUTS:      None
*
* NOTES:        Needs to be run in setup.
*
*/  
void guiInit()
{
  gui = new ControlP5(this);
  slider1 = gui.addSlider("CPU Fan Control")
               .setPosition(20, 20)
               .setSize(300, 20)
               .setRange(0,255)
               .setDecimalPrecision(0) 
               ;
  slider2 = gui.addSlider("GPU Fan Conrtol")
               .setPosition(20,50)
               .setSize(300,20)
               .setRange(0,255)
               .setDecimalPrecision(0) 
               ;
}
  


/*****************************************************
* NAME:         sendSpeed
*
* DESCRIPTION:  Sends serial packet to ID a fan and its speed.
*
*
* INPUTS:       2 Two bytes of data to send, the fan number ID and its speed.
* 
* OUTPUTS:      None
*
* NOTES:        Use this function to directly control a fan. 
*
*/  
void sendSpeed(byte fan, byte speed)
{
  myPort.write(fan);
  delay(10);
  myPort.write(speed);
  delay(10);
}


/*****************************************************
* NAME:         serialEvent
*
* DESCRIPTION:  Reads incoming serial data and parses the fan speeds.
*
*
* INPUTS:       Serial port object.
* 
* OUTPUTS:      None
*
* NOTES:        Does not need to be added to the main loop. Acts like an interrupt
*               and triggers upon a serial receive event. 
*
*/  
void serialEvent(Serial myPort) 
{ 
  int idBuffer = myPort.read();
  if(idBuffer == 0x01)
  {
    cpuRX = myPort.read();
  }  
  else if(idBuffer == 0x02)
  {
    gpuRX = myPort.read();
  }
} 


/*****************************************************
* NAME:         serialInit
*
* DESCRIPTION:  Initializes serial port to operate at the specified baudrate.
*
*
* INPUTS:       Baudrate
* 
* OUTPUTS:      None
*
* NOTES:        Needs to be run in setup. 
*
*/  
void serialInit(int baudRate)
{
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], baudRate); 
  myPort.buffer(2);
}