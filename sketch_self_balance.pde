import processing.serial.*;
// The serial port:
Serial serial;       
Handle h = new Handle(200,200,20,200);
GraphicalLogger gl = new GraphicalLogger(20,20);
void setup()
{
  
// Open the port you are using at the rate you want:
  serial = new Serial(this, Serial.list()[0], 115200);
  size(800,600,P2D);
}
float r = 0.0;
void draw()
{
  if (serial.available() > 0) {
    String inBuffer = serial.readString();   
    r = float(inBuffer);    
  }
  r += 0.5; 
  gl.add("angle",r);
  background(255,255,255);
  h.draw(radians(r));
  gl.draw();
}
