import processing.serial.*;

Serial myPort;
String inString;
float roll, pitch, yaw;

PShape plane;

void setup() {
  size(800, 600, P3D); // 3D canvas
  myPort = new Serial(this, "COM3", 115200); // change COM port!
  myPort.bufferUntil('\n');
  
  plane = loadShape("plane.obj");
}

void draw() {
  background(200);
  lights();

  translate(width/2, height/2, 0); 
  scale(3.0); // zoom bigger
  
  // fix the model’s initial orientation
  rotateX(HALF_PI);   // rotate 90° around X
  rotateZ(PI);        // flip upside down if needed

  rotateY(radians(yaw));    // yaw about vertical
  rotateX(radians(pitch));  // pitch nose up/down
  rotateZ(radians(roll));   // roll wing up/down
  
  shape(plane);
}

void serialEvent(Serial myPort) {
  inString = trim(myPort.readStringUntil('\n'));
  if (inString != null) {
    String[] list = split(inString, ',');
    if (list.length == 3) {
      roll = float(list[0]);
      pitch = float(list[1]);
      yaw = float(list[2]);
    }
  }
}
