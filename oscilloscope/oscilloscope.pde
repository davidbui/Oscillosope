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

// * ------------------ Generating a sin wave ------------------
int xspacing = 16;           // How far apart should each horizontal location be spaced
int w;                       // Width of entire wave
float theta = 0.0;           // Start angle at 0
float amplitude = 15;        // Height of wave
float period = 500.0;        // How many pixels before the wave repeats
float dx;                    // Value for incrementing X, a function of period and xspacing
float[] yvalues;             // Using an array to store height values for the wave
float yvalue;                // Store the value of the sin wave
// * ----------------------------------------------

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
final char TRIGGER    = 't'; // Trigger
// * ----------------------------------------------

// * --------------- STARTING STATE ---------------
float zoom    = 1.0;
float scale   = 1.0;
int centerV   = 0;
int centerH   = 0;
int gridLines = 0;
int com_port  = 0;         // Index number in Serial.list
int baud_rate = 115200;
int pHigh;                 // pixel location of the upper voltage bar (15V)
int pLow;                  // pixel location of the lower voltage bar (-15V)
float vhigh = 15;          // Starting upper voltage level.
float vlow = -15;          // Starting lower voltage level.
// * ----------------------------------------------

// Text labels.
Textlabel myTextlabelA;

// GUI Box.
int boxWidth = 1200;
int boxLength = 700;
int boxMain  = boxWidth-200;

// Global Variables.
ControlP5 cp5;             // GUI lib.
Serial port;               // Create object from Serial class.
PFont f;

String rcvBuf;             // Buffer to hold messages received from the serial port.
float data;                // Holds the data from the serial port.
int[] timeBars = {0,0};    // The horizontal line boundaries.
long[] times;              // Holds the the current times.
float[] fvalues;           // Holds the voltage readings. 
int   timeMode = 0;        // 0 = off, 1 = first veritcal bar, 2 = second veritcal bar.
boolean pause;             // To pause the screen.
//float voltage;

// * --------------- TRIGGERING ---------------
boolean isTriggered;      // Is trigger mode on or off.
boolean foundTrigger;     // Have we found a trigger point yet?
float triggerLevel;       // Current trigger level.
float[] valuesTriggered;  // Holds the buffer to print.
int triggeredPassed;      // Sets where the trigger will be. (Default half the screen)
float trigLevel;          // The current trigger level.
float triggerTime;        // The time at which the trigger is found.
float triggerFrequency;
// * ----------------------------------------------


void setup()
{
  foundTrigger = false;
  triggeredPassed = 0;
  triggerTime = 0;

  frameRate(1000);
  
  // sin wave
  w = boxMain+16;
  dx = (TWO_PI / period) * xspacing;
  yvalues = new float[w/xspacing];
  
  size(1200, 700, P2D);
  f = createFont("Arial", 16, true); 

  printArray(Serial.list());
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
  // values = new int[boxMain];
  fvalues = new float[boxMain];
  timeBars[0] = boxMain/3;
  timeBars[1] = 2*boxMain/3;
  isTriggered = false;

  triggerLevel = height/3;

    // Get scaled values for bounds
  pHigh = getY(1023);
  pLow  = getY(0);
  valuesTriggered = new float[boxMain];

}

// * ------------------ BUTTON LOGIC ------------------
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
// * ----------------------------------------------

// * ------------------ MISC FUNCTIONS ------------------
// Returns a value read from the serial port until a '\n' character is received. Returns -1 if '\n' not found.
float getValue() {
  float value = -1;
  // Check if data is available in the port.
  if (port.available() > 0) { 
      rcvBuf = port.readStringUntil('\n'); 
    if (rcvBuf != null) {
      // A newline was found.
      value = float(rcvBuf);
    }
  }
  return value;
}

// Returns a pixel value given a voltage value. (does not change with scale)
int getY(int val) {
  return (int)(height/2 -(val-512)*0.8 / 1023.0f * (height - 1));
}

// Returns a pixel value given a voltage value.
float getYFloat(float val) {
  return (float)(height/2 - ((pLow-pHigh)/(2*vhigh))*val*scale);
}

// Returns a voltage value given a pixel value.
float getVoltage(float val) {
  float half = (pLow-pHigh)/2;
  return (((height/2)- val)*(vhigh/half));
}

// Draw voltage waveforms.
void drawLines() {
  int x0 = 0, x1 = 0;
  float y0 = 0, y1 = 0;
  stroke(255,255,0);

  for (int i=0; i<boxMain; i++) {
    x1 = round(boxMain - ((boxMain-i) * zoom) + centerH);

    // Check if it is in triggered mode.
    if (isTriggered) {
      // A trigger is found and the trigger point is currently at the middle of the screen.
      if (foundTrigger && triggeredPassed == 0)
      {
        arrayCopy(fvalues, valuesTriggered, boxMain);
        foundTrigger = false;
      }
        y1 = getYFloat(valuesTriggered[i]);
    } else {
      y1 = getYFloat(fvalues[i]);
    }

    // Only display waveform in bounds of the 'screen'.
    if(i > 1 && y1 <= pLow && y1 >= pHigh && y0 <= pLow && y0 >= pHigh)
      line(x0, y0, x1, y1);
    x0 = x1;
    y0 = y1;
  }
}

// Draw gridlines (bounds, minor).
void drawGrid() {
  // Draw voltage bounds
  stroke(255, 0, 0);
  line(0, pHigh, boxMain, pHigh);
  line(0, pLow, boxMain, pLow);

  // Add voltage bound text
  textFont(f, 20);
  fill(255, 0, 0);
  
  text(truncate(vhigh,1)+"V", 5, pHigh-2);
  text(truncate(vlow,1)+"V", 5, pLow+18);

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
  
  // Print frequency in triggered mode.
  if (isTriggered) {
    textFont(f, 16);
    fill(204, 102, 0);
    triggerFrequency = 1/((System.nanoTime() - triggerTime)/1000000000);
    text("Frequency: " + triggerFrequency + "Hz", boxMain/2, height*0.95);
  }
}

// Draw vertical 'time bars' (seperate from above for better layering).
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

// Push a voltage value into the data array.
void pushValue(float value) {
  for (int i=0; i<boxMain-1; i++) {
    fvalues[i] = fvalues[i+1];
  }
  fvalues[boxMain-1] = value;

  if (triggeredPassed > 0)
    triggeredPassed--;
}

// Truncate a floating point number.
float truncate(float x, int digits) {
  float temp = pow(10.0, digits);
  return round( x * temp ) / temp;
}

// Generates a sin wave.
void calcWave() {
  // Increment theta (try different values for 'angular velocity' here
  theta += 0.02;

  // For every x value, calculate a y value with sine function
  float x = theta;
  //for (int i = 0; i < yvalues.length; i++) {
    yvalue = sin(x)*amplitude + 0;
    x+=dx;
  //}
}

void trigger()
{
  // Only look for a trigger if a trigger has not been found yet.
  if (!foundTrigger) {
    // Look for a rising edge trigger.
    if (fvalues[boxMain-1] > trigLevel && fvalues[boxMain-2] < trigLevel)
    {
      // println("Found the trigger!");
      foundTrigger = true;
      triggeredPassed = boxMain/2;      // Show waveform when the trigger point reaches the middle of the screen.
      triggerTime = System.nanoTime();  // Store the current time to measure the frequency of the signal.
    }
  }
}
// * ----------------------------------------------

// When a key is pressed down or held.
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
  case BAR_RIGHT:                                            // Move the time bar right (also mouse click)=
    if (timeMode == 1 && timeBars[0] < boxMain-1)
      timeBars[0] += 1;
    else if (timeMode == 2 && timeBars[1] < boxMain-1)
      timeBars[1] += 1; 
    break;
  case TRIGGER:
    if (isTriggered) {
      println("Turn off trigger");
      isTriggered = false;
      break;
    } else
      isTriggered = true;
      println("Turn on trigger");
      trigLevel = getVoltage(triggerLevel);
      println("trigLevel = " + trigLevel);
    break;
  }  
}

// When a key is released.
void keyReleased() {
  println(key+": "+(int)key);
  switch (key) {
  case Z_IN:                                                 // Zoom horizontal in
    zoom *= 2.0f;
    if ( (int) (boxMain / zoom) <= 1 )
      zoom /= 2.0f;
    break;
  case Z_OUT:                                                // Zoom horizontal out
    zoom /= 2.0f;
    if (zoom < 1.0f)
      zoom *= 2.0f;
    break;
  case S_IN:                                                 // Scale vertical in
    scale*=2;
    vhigh/=2;
    vlow/=2;
    break;                                                  
  case S_OUT:                                                // Scale vertical out
    scale /= 2; 
    vhigh*=2;
    vlow*=2;
  break;                             
  case RESET_AXIS:                                           // Reset all scaling
    centerV = 0; centerH = 0;
    scale = 1.0; zoom  = 1; gridLines = 0;
    break;
  case MEAS_TIME: timeMode = (timeMode + 1) % 3; break;      // Change the vertical bars (off, left bar, right bar)
  case TOG_PAUSE:                                            // Toggle waveform pausing
    if (pause) {
      centerH = 0;
      for (int i=0; i<boxMain; i++){
        fvalues[i] = 0;                                       // Clear data on resume
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
  for(int i=0; i<600; i++) {
    background(0);  // Blackground.

    // Get current voltage, time of reading
    //data = getValue();
    calcWave();
    data = yvalue;
  
    // Update the current data buffer if it is not paused and there is data in the serial.
    if (!pause && data != -1) {
      // print(data+"\n");
      pushValue(data);
      
      // Print current voltage reading
      /*
      textFont(f, 16);
      fill(204, 102, 0);
      voltage = truncate(5.0*data / 1023, 1);
      text("Voltage: " + voltage + "V", 1170, 30);
      */
    }

    // Only trigger in trigger mode.
    if (isTriggered) {
       trigger(); 
    }

    drawGrid();      // Draw main gui lines (horizontal voltage lines, vertical line seperator, etc).    
    drawLines();     // Draw the voltage waveforms.
    drawVertLines(); // Vertical line measurements.
  }

}