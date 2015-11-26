/*
* Oscilloscope
 * Gives a visual rendering of analog pin in realtime.
 *
 * ---------------- IMPROVEMENTS ------------------
 * Updates by John Porter, 2/7/2014
 * Added ability to move waveform left or right.
 * Added gridlines (bounds and minor).
 * Added ability to pause/resume.
 * Added ability to measure time.
 * General usability improvements.
 *
 * --------------- ORIGINAL PROJECT ---------------
 * This project is part of Accrochages
 * See http://accrochages.drone.ws
 * (c) 2008 Sofian Audry (info@sofianaudry.com)
 * ------------------------------------------------
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

import processing.serial.*;
import controlP5.*;

// Sin wave

int xspacing = 16;   // How far apart should each horizontal location be spaced
int w;              // Width of entire wave

float theta = 0.0;  // Start angle at 0
float amplitude = 510;  // Height of wave
float period = 500.0;  // How many pixels before the wave repeats
float dx;  // Value for incrementing X, a function of period and xspacing
float[] yvalues;  // Using an array to store height values for the wave
float yvalue;

// * ------------------ HOT KEYS ------------------
final char T_UP       = 'w'; // Translate waveform up
final char T_DOWN     = 's'; //                    down
final char T_LEFT     = 'a'; //                    left
final char T_RIGHT    = 'd'; //                    right
final char Z_IN       = 'c'; // Horizontal zoom in
final char Z_OUT      = 'z'; //                 out
final char S_IN       = 'e'; // Vertical scale in
final char S_OUT      = 'q'; //                out
final char MGL_UP     = 'r'; // Minor grid lines increase
final char MGL_DOWN   = 'f'; //                  decrease
final char TOG_PAUSE  = 'p'; // Toggle pause (unpause resets waveform)
final char RESET_AXIS = ' '; // Reset axis settings
final char MEAS_TIME  = 'x'; // Adds and/or highlights vertical bars (time measurement)
final char BAR_LEFT   = ','; // Move highlighted vertical bar left (can also mouse click)
final char BAR_RIGHT  = '.'; //                               right
// * ----------------------------------------------

// * --------------- STARTING STATE ---------------
float zoom    = 1.0;
float scale   = 0.5;
int centerV   = 0;
int centerH   = 0;
int gridLines = 0;
int com_port  = 0;   // Index number in Serial.list
int baud_rate = 115200;
// * ----------------------------------------------

// Text labels.
Textlabel myTextlabelA;

// GUI Box.
int boxWidth = 1000;
int boxLength = 480;
int boxMain  = boxWidth-200;


// Global Variables.
ControlP5 cp5;             // GUI lib.
Serial port;               // Create object from Serial class
String rcvBuf;             // Buffer to hold messages sent from mcu
int[] timeBars = {0,0};    // The horizontal line boundaries
long[] times;
int[] values;
float[] vals;
int   timeMode = 0;
PFont f;
boolean pause;
float val;                        // Data received from the serial port
long valTime;                   // Time data was received
float voltage;

void setup()
{
  // sin wave
  w = boxMain+16;
  dx = (TWO_PI / period) * xspacing;
  yvalues = new float[w/xspacing];
  
  size(1000, 480);
  f = createFont("Arial", 16, true);

  port = new Serial(this, Serial.list()[com_port], baud_rate);
  cp5 = new ControlP5(this);

  // Setup text labels.
  myTextlabelA = cp5.addTextlabel("label")
                  .setText("Trigger")
                  .setPosition(width-180,20)
                  .setColorValue(255)
                  .setFont(createFont("Georgia",15))
                  ;
  
  // Setup buttons.
  cp5.addButton("ch1")
     .setValue(0)
     .setPosition(boxWidth-180,40)
     .setSize(30,19)
     ;
  
  cp5.addButton("Signal_Off")
     .setValue(1)
     .setPosition(boxWidth-180,60)
     .setSize(55,19)
     ;
  
  times = new long[boxMain];
  values = new int[boxMain];
  vals = new float[boxMain];
  timeBars[0] = boxMain/3;
  timeBars[1] = 2*boxMain/3;
 
}


// BUTTON LOGIC

// This function captures any button actions.
public void controlEvent(ControlEvent theEvent) {
  println(theEvent.getController().getName());
}

public void ch1(int theValue) {
  println("a button event from ch1: "+theValue);
}

public void Signal_Off(int theValue) {
  println("a button event from Signal_Off: "+theValue);
}


// Misc functions

// Read value from serial stream
int getValue() {
  int value = -1;
  while (port.available () >= 3) {
    if (port.read() == 0xff) {
      value = (port.read() << 8) | (port.read());
    }
  }
  return value;
}

// Get a y-value for the datapoint, varies based on axis settings
int getY(int val) {
  return (int)(height/2 -(val-512+centerV)*scale / 1023.0f * (height - 1));
}

// Get a y-value for the datapoint, varies based on axis settings
float getYFloat(float val) {
  return (float)(height/2 -(val-512+centerV)*scale / 1023.0f * (height - 1));
}

// Draw voltage waveforms.
void drawLines() {
  int x0 = 0, x1 = 0;
  float y0 = 0, y1 = 0;
  stroke(255,255,0);
  for (int i=0; i<boxMain; i++) {
    x1 = round(boxMain - ((boxMain-i) * zoom) + centerH);
    //y1 = getY(values[i]);
    y1 = getYFloat(vals[i]);
    if(i > 1)
      line(x0, y0, x1, y1);
    x0 = x1;
    y0 = y1;
  }
}

// Draw gridlines (bounds, minor)
void drawGrid() {
  // Get scaled values for bounds
  int pFive = getY(1023);
  int zero  = getY(0);
//println("pfive" + pFive);
//println("zero" + zero);
  // Draw voltage bounds
  stroke(255, 0, 0);
  line(0, pFive-1, boxMain, pFive-1);
  line(0, zero+1, boxMain, zero+1);

  // Add voltage bound text
  textFont(f, 10);
  fill(255, 0, 0);
  text("+5V", 5, pFive+12);
  text(" 0V", 5, zero-4);

  // Draw minor grid lines
  int gridVal = 0;
  stroke(75, 75, 75);
  for (int i = 0; i < gridLines; i++) {
    gridVal = getY(round((i+1.0)*(1023.0 / (gridLines+1.0))));
    line(0, gridVal, boxMain, gridVal);
  }

  // Add minor grid line text
  if (gridLines > 0) {
    textFont(f, 16);
    fill(204, 102, 0);
    float scaleVal = truncate(5.0f / (gridLines+1), 3);
    text("Grid: " + scaleVal + "V", 1170, height-12);
  }
  
  // Print difference between vertical 'time' bars
  if (timeMode > 0) {
    textFont(f, 16);
    fill(204, 102, 0);
    
    int idx0 = round(boxMain + (timeBars[0] - boxMain - centerH)/zoom);
    int idx1 = round(boxMain + (timeBars[1] - boxMain - centerH)/zoom);
    
    // Ensure time bars are over a recorded portion of the waveform
    if(idx1 < 0 || idx0 < 0 || idx1 > (boxMain-1) || idx0 > (boxMain-1) || times[idx1] == 0 || times[idx0] == 0)
      text("Time: N/A", 30, height-12);
    else{
      float timeDiff = truncate((times[idx1] - times[idx0])/2000000.0,2);
      text("Time: " + timeDiff + "ms", 30, height-12);
    }
  }
  
  // Line divisor b/w signal and buttons.
  line(boxMain, 0, boxMain, boxLength);
}

// Draw vertical 'time bars' (seperate from above for better layering)
void drawVertLines(){
  stroke(75, 75, 75);
  if (timeMode == 1) {
    line(timeBars[1], 0, timeBars[1], height);
    stroke(100, 100, 255);
    line(timeBars[0], 0, timeBars[0], height);
  }
  else if (timeMode == 2) {
    line(timeBars[0], 0, timeBars[0], height);
    stroke(100, 255, 100);
    line(timeBars[1], 0, timeBars[1], height);
  }
}

// Push the values in the data array
void pushValue(float value) {
  for (int i=0; i<boxMain-1; i++)
    vals[i] = vals[i+1];
  vals[boxMain-1] = value;
}

// Push the timestamps in the time array
void pushTime(long time) {
  for (int i=0; i<boxMain-1; i++)
    times[i] = times[i+1];
  times[boxMain-1] = time;
}

// Truncate a floating point number
float truncate(float x, int digits) {
  float temp = pow(10.0, digits);
  return round( x * temp ) / temp;
}


// When a key is pressed down or held...
void keyPressed() {
  switch (key) {
  case T_UP: centerV += 10/scale; break;                     // Move waveform up
  case T_DOWN: centerV -= 10/scale; break;                   // Move waveform down
  case T_RIGHT: centerH += 10/scale; break;                  // Move waveform right
  case T_LEFT: centerH -= 10/scale; break;                   // Move waveform left
  case MGL_UP:                                               // Increase minor grid lines
    if (gridLines < 49)
      gridLines += 1;
    break;
  case MGL_DOWN:                                             // Decrease minor grid lines
    if (gridLines > 0)
      gridLines -= 1;
    break;
  case BAR_LEFT:                                             // Move the time bar left (also mouse click)
    if (timeMode == 1 && timeBars[0] > 0)
      timeBars[0] -= 1;
    else if (timeMode == 2 && timeBars[1] > 0)
      timeBars[1] -= 1; 
    break;
  case BAR_RIGHT:                                            // Move the time bar right (also mouse click)
    if (timeMode == 1 && timeBars[0] < boxMain-1)
      timeBars[0] += 1;
    else if (timeMode == 2 && timeBars[1] < boxMain-1)
      timeBars[1] += 1; 
    break;
  }
}

// When a key is released...
void keyReleased() {
  println(key+": "+(int)key);
  switch (key) {
  case Z_IN:                                                 // Zoom horizontal
    zoom *= 2.0f;
    if ( (int) (boxMain / zoom) <= 1 )
      zoom /= 2.0f;
    break;
  case Z_OUT:                                                // Zoom horizontal
    zoom /= 2.0f;
    if (zoom < 1.0f)
      zoom *= 2.0f;
    break;
  case S_IN: scale*=2; break;                                // Scale vertical
  case S_OUT: scale /= 2; break;                             // Scale vertical
  case RESET_AXIS:                                           // Reset all scaling
    centerV = 0; centerH = 0;
    scale = 0.5; zoom  = 1; gridLines = 0;
    break;
  case MEAS_TIME: timeMode = (timeMode + 1) % 3; break;      // Change the vertical bars (off, left bar, right bar)
  case TOG_PAUSE:                                            // Toggle waveform pausing
    if (pause) {
      centerH = 0;
      for (int i=0; i<boxMain; i++){
        vals[i] = 0;                                       // Clear data on resume
        times[i] = 0;
      }
    }
    pause = !pause;
  }
}

// Use mouse clicks to quickly move vertical bars (if highlighted)
void mousePressed() {
  if(timeMode == 1)
    timeBars[0] = mouseX;
  else if(timeMode == 2)
    timeBars[1] = mouseX;
}

void draw()
{
  // Blackground.
  background(0);

  // PROGRAM LOGIC
  calcWave();
  //println(yvalue);

  // Draw main gui lines (horizontal voltage lines, vertical line seperator, etc).
  drawGrid();
  stroke(126);

  // Get current voltage, time of reading
  val = getValue();
  // madhax
  //val = yvalue;
  print(val+"\n");
  valTime = System.nanoTime();
  
  // If not paused
  if (!pause && val != -1) {
    // Push value/time onto array
    pushValue(val);
    pushTime(valTime);
    
    // Print current voltage reading
    textFont(f, 16);
    fill(204, 102, 0);
    voltage = truncate(5.0*val / 1023, 1);
    text("Voltage: " + voltage + "V", 1170, 30);
  }
  

  // Draw the voltage waveforms.
  drawLines();
  //drawVertLines();
}

void calcWave() {
  // Increment theta (try different values for 'angular velocity' here
  theta += 0.02;

  // For every x value, calculate a y value with sine function
  float x = theta;
  //for (int i = 0; i < yvalues.length; i++) {
    yvalue = sin(x)*amplitude + 510;
    x+=dx;
  //}
}