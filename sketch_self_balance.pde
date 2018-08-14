
import grafica.*;
import processing.serial.*;
// The serial port:
Serial serial;       
Handle h = new Handle(200, 200, 10, 100);
GraphicalLogger gl = new GraphicalLogger(20, 20);

Plot pAngle;
Plot pDAngle;
void setup()
{

  // Open the port you are using at the rate you want:
  serial = new Serial(this, "/dev/ttyUSB0", 115200);
  size(800, 600, P2D);
  pAngle = new Plot(500, 0, 200, 100, "Angle", this);
  pDAngle = new Plot(500, 200, 200, 100, "dAngle", this);
}
float r = 0.0;
int i = 0;
float last = 0;
float r_last=0;
void draw()
{
  while (serial.available() > 0) {
    background(255, 255, 255);
    String inBuffer = serial.readStringUntil('\n'); 
    if(inBuffer == null)
      break;
    println(inBuffer);

    if (inBuffer != null) 
      r = float(inBuffer);    
    if(r > 0)
      r = 180-r;
    else
      r = -180-r;
    float now = millis()/1000.0;
    float dAngle  = (r-r_last)/(now-last);
    //r += 0.5; 
    
    gl.add("angle", r);
    gl.add("dAngle", dAngle);
    

    pAngle.addPoint(now,r);
    pDAngle.addPoint(now,dAngle);


    h.draw(-radians(r+180));
    gl.draw();
    pAngle.draw();
    pDAngle.draw();
    last = now;
    r_last = r;
  }
}
