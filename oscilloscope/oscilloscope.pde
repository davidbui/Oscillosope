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
float amplitude = 15;  // Height of wave
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
int com_port  = 1;   // Index number in Serial.list
int baud_rate = 115200;
// * ----------------------------------------------

// Text labels.
Textlabel myTextlabelA;
Textarea myTextArea;
// GUI Box.
int boxWidth = 2000;
int boxLength = 1200;
int boxMain  = boxWidth-int(boxWidth*0.2);


// Global Variables.
ControlP5 cp5;             // GUI lib.
Serial port;               // Create object from Serial class.

String rcvBuf;             // Buffer to hold messages sent from microcontroller.
int[] timeBars = {0,0};    // The horizontal line boundaries.
long[] times;              // Holds the the current times.
//int[] values;            // For the int original.
float[] fvalues;           // Holds the voltage readings. 
int   timeMode = 0;
PFont f;
boolean pause;             // To pause the screen.
float val;                        // Data received from the serial port
float voltage;
long start, end;
boolean isTriggered;      // Is trigger mode on or off.
boolean foundTrigger;     // Have we found a trigger point yet?
float triggerLevel;
float[] valuesTriggered;  // holds the buffer to print.
int triggeredPassed;
float trigLevel;          // The current trigger level.
float triggerTime;
float triggerFrequency;
float newTriggerFrequency;
int x = 100;
float temptrigger;
// Get scaled values for bounds
int high;
int low;
int ohigh;
int olow;
float vhigh = 15;
float vlow = -15;
int ad;
int as;

void setup()
{
  ad = as = 0;
  temptrigger = 0;
  foundTrigger = false;
  triggeredPassed = 0;
  triggerTime = 0;

  frameRate(1000);
  
  // sin wave
  w = boxMain+16;
  dx = (TWO_PI / period) * xspacing;
  yvalues = new float[w/xspacing];
  
  size(2000, 1200, P2D);
  f = createFont("Arial", 16, true); 

  printArray(Serial.list());
 // port = new Serial(this, Serial.list()[com_port], baud_rate);
  cp5 = new ControlP5(this);

  // Setup text labels.
  myTextlabelA = cp5.addTextlabel("label")
                  .setText("Osciloscope")
                  .setPosition(((width-boxMain)/2)-110+boxMain,20)
                  .setColorValue(255)
                  .setFont(createFont("Georgia",40))
                  ;
  
  myTextArea = cp5.addTextarea("txt")
                  .setPosition(boxMain+5,100)
                  .setSize(width-boxMain,height)
                  .setFont(createFont("arial",30))
                  .setLineHeight(30)
                  .setColor(color(255))
                  .setColorBackground(color(0,0))
                  .setColorForeground(color(0,0));
                  ;
  
  myTextArea.setText("               HOTKEYS\n\n      Waveform translation\n                 w - up\n                 d - down\n                 a- left\n                 d - right"
                 + "\n\n        Horizontal zoom\n                   c - in\n                   z - out"
                 + "\n\n            Vertical scale\n                  e - in\n                  q - out"
                 + "\n\n               Gridlines\n                r - add\n                f - remove"
                 + "\n\n                TRIGGER\n         t - Toggle ON/OFF"
                  );
  
                  
  printArray(Serial.list());
  //port = new Serial(this, Serial.list()[com_port], baud_rate);
  cp5 = new ControlP5(this);



  times = new long[boxMain];
  // values = new int[boxMain];
  fvalues = new float[boxMain];
  timeBars[0] = boxMain/3;
  timeBars[1] = 2*boxMain/3;
  isTriggered = false;

  triggerLevel = height/4;

    // Get scaled values for bounds
  ohigh = high = getY(1023);
  olow = low  = getY(0);
  valuesTriggered = new float[boxMain];

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
float getValue() {
  float value = -1;
  if (port.available() > 0) { 
    //  until we get a linefeed
      rcvBuf = port.readStringUntil('\n'); 
    // Convert the byte array to a String
    if (rcvBuf != null)
      value = float(rcvBuf);
  }

  return value;
}

// Get a y-value for the datapoint, varies based on axis settings
int getY(int val) {
  return (int)(height/2 -(val-512)*0.8 / 1023.0f * (height - 1));
}

// Get a y-value for the datapoint, varies based on axis settings
/*float getYFloat(float val) {
  return (float)(height/2 -(val+centerV)*scale / 1023.0f * (height - 1));
}*/

float getYFloat(float val) {
  return (float)(height/2 - ((low-high)/(2*vhigh))*val*scale);
}

float getVoltage(float val) {
  float half = (low-high)/2;
  
  return (((height/2)- val)*(vhigh/half));
  
 // return (float)(height/2 - ((low-high)/(2*15))*val*scale);
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
      if (foundTrigger && triggeredPassed <= 0)
      {
        arrayCopy(fvalues, valuesTriggered, boxMain);
        foundTrigger = false;
      }
        y1 = getYFloat(valuesTriggered[i]);
    } else {
      y1 = getYFloat(fvalues[i]);
    }

    if(i > 1 && y1 <= olow && y1 >= ohigh && y0 <= olow && y0 >= ohigh)
      line(x0, y0, x1, y1);
    x0 = x1;
    y0 = y1;
  }
}

// Draw gridlines (bounds, minor)
void drawGrid() {
  // Draw voltage bounds
  stroke(255, 0, 0);
  line(0, high, boxMain, high);
  line(0, low, boxMain, low);

  // Add voltage bound text
  textFont(f, 20);
  fill(255, 0, 0);
  
  text(truncate(vhigh,1)+"V", 5, high-2);
  text(truncate(vlow,1)+"V", 5, low+18);

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
    triggerFrequency = truncate(1/((System.nanoTime() - triggerTime)/1000000000),1);
    if(x-- == 0) {
      newTriggerFrequency = triggerFrequency;
      x = 200;
    }
    text("Frequency: " + newTriggerFrequency + "Hz", boxMain/2+50, height*0.95);
    line(boxMain-temptrigger, 0, boxMain-temptrigger, height);
  }
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
    fvalues[i] = fvalues[i+1];
  fvalues[boxMain-1] = value;
  triggeredPassed--;
}


// Truncate a floating point number
float truncate(float x, int digits) {
  float temp = pow(10.0, digits);
  return round( x * temp ) / temp;
}


// When a key is pressed down or held...
void keyPressed() {
  switch (key) {
  case T_UP:                      // Move waveform up
    if (isTriggered) {
      as -= 25;
    } else {
      centerV += 10/scale;
    }
    break;
  case T_DOWN:                    // Move waveform down
    if (isTriggered) {
      as += 25;
    } else {
      centerV -= 10/scale;
    }
    break;
  case T_RIGHT:                   // Move waveform right
    if (isTriggered) {
      ad -= 50;
    } else {
      centerH += 10/scale; 
    }
    
    break;
  case T_LEFT:                    // Move waveform left
    if (isTriggered) {
      ad += 50;
    } else {
      centerH -= 10/scale; 
    }
    break;
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
      trigLevel = getVoltage(triggerLevel+as);
      println("trigLevel = " + trigLevel);
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
  case S_IN:
    scale*=2; 
    vhigh/=2;
    vlow/=2;
    break;                                // Scale vertical
  case S_OUT:
    scale /= 2; 
    vhigh*=2;
    vlow*=2;
  break;                             // Scale vertical
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

void draw()
{
  for(int i=0; i<60; i++) {
    background(0);  // Blackground.

    // Get current voltage, time of reading
    //val = getValue();
    calcWave();
    val = yvalue;
  
    // If not paused and there is data in serial.
    if (!pause && val != -1) {
     // print(val+"\n");
      pushValue(val);
      
    }
  
    if (isTriggered) {
       trigger(); 
    }

    // Draw main gui lines (horizontal voltage lines, vertical line seperator, etc).    
    drawGrid();
    drawLines();     // Draw the voltage waveforms.
    drawVertLines(); // Vertical line measurements.
  }

}



void trigger()
{
  // Looking for a trigger..
  if (!foundTrigger) {
    // Found the newest point is on trigger level.
    if (fvalues[boxMain-1] > (trigLevel*scale) && fvalues[boxMain-2] < (trigLevel*scale)) {      //println("Found trigger point");
      // Rising edge trigger.
      if (fvalues[boxMain-1] > fvalues[boxMain-2]) {
        println("Found the trigger!");
        foundTrigger = true;

        // Show waveform when the trigger point reaches the middle of the screen.
        triggeredPassed = boxMain/2 +ad;
        temptrigger = triggeredPassed;
        triggerTime = System.nanoTime();
        
      }
    }
    
  }
}