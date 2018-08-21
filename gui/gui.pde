/*
Drone Motor Analysis Tool

  gui.pde

  Interfaces with arduino controller over serial data
  
  Created 18 May 2018
  by James Hodgson
*/

import processing.serial.*;
import controlP5.*;
import java.util.*;

static ControlP5 cp5;
Serial myPort;
Chart thrustChart, currentChart;
Table results;

String dataStr = "NO DATA - PLEASE REFRESH THE LIST AND SELECT THE CORRECT COM PORT";
float time, prevTime, dTime;
float voltage, throttle, idleThrottle, runThrottle, testStep, testElapsed;
float thrust,  minThrust, avgThrust, maxThrust;
float current, minCurrent, avgCurrent, maxCurrent;
int test, prevTest;
color white = color(255,255,255);
color lgray = color(200,200,200);
color dgray = color(120,120,120);
color bgray = color(90,109,126);
color black = color(0,0,0);
color dblue = color(0,45,90);
color lblue = color(0,116,217);
color vlblue = color(0,170,255);
List ports;

void setup () {
  println(Serial.list());                               // List all the available serial ports
  ports = Arrays.asList(Serial.list());
  
  size(960, 720);
  frameRate(200);
  textFont(createFont("Arial",16,true),16);
  cp5 = new ControlP5(this);
  cp5.setAutoDraw(false);
  cp5.addToggle("armToggle")
          .setBroadcast(false)
          .setValue(false)
          .setPosition(8,8)
          .setSize(200,100)
          .setFont(createFont("Agency FB Bold",48,true))
          .setCaptionLabel("ARM")
          .setColorForeground(color(255,100,100))
          .setColorBackground(lblue)
          .setColorActive(color(220,0,0))
          .setColorCaptionLabel(white)
          .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER);
  cp5.addToggle("idleToggle")
          .setBroadcast(false)
          .setValue(false)
          .setPosition(8,124)
          .setSize(200,40)
          .setFont(createFont("Agency FB Bold",24,true))
          .setCaptionLabel("IDLE")
          .setColorForeground(color(100,255,100))
          .setColorBackground(bgray)
          .setColorActive(color(0,220,0))
          .setLock(true)
          .setColorCaptionLabel(white)
          .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER);
  cp5.addToggle("runToggle")
          .setBroadcast(false)
          .setValue(false)
          .setPosition(8,172)
          .setSize(200,40)
          .setFont(createFont("Agency FB Bold",24,true))
          .setCaptionLabel("RUN")
          .setColorForeground(color(100,255,100))
          .setColorBackground(bgray)
          .setColorActive(color(0,220,0))
          .setLock(true)
          .setColorCaptionLabel(white)
          .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER);
  cp5.addToggle("testToggle")
          .setBroadcast(false)
          .setValue(false)
          .setPosition(8,220)
          .setSize(200,40)
          .setFont(createFont("Agency FB Bold",24,true))
          .setCaptionLabel("TEST")
          .setColorForeground(color(100,255,100))
          .setColorBackground(bgray)
          .setColorActive(color(0,220,0))
          .setLock(true)
          .setColorCaptionLabel(white)
          .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER);
  cp5.addSlider("idleSlider")
          .setBroadcast(false)
          .setValue(0)
          .setPosition(216,134)
          .setSize(400,20)
          .setFont(createFont("Agency FB Bold",16,true))
          .setRange(0,10)
          .setNumberOfTickMarks(41)
          .setCaptionLabel("IDLE THROTTLE % (START)");
  cp5.addSlider("runSlider")
          .setBroadcast(false)
          .setValue(0)
          .setPosition(216,182)
          .setSize(400,20)
          .setFont(createFont("Agency FB Bold",16,true))
          .setRange(0,100)
          .setNumberOfTickMarks(21)
          .setCaptionLabel("RUN THROTTLE % (END)");
  cp5.addSlider("testSlider")
          .setBroadcast(false)
          .setValue(0)
          .setPosition(216,230)
          .setSize(400,20)
          .setFont(createFont("Agency FB Bold",16,true))
          .setRange(0,4)
          .setNumberOfTickMarks(41)
          .setCaptionLabel("STEP DURATION (s)");
  cp5.addTextfield("batteryTitle")
          .setPosition(217,8)
          .setSize(114,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(white)
          .setColorForeground(dblue)
          .setColorBackground(dblue)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("BATTERY")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("batteryValue")
          .setPosition(217,44)
          .setSize(114,64)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("throttleTitle")
          .setPosition(341,8)
          .setSize(114,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(white)
          .setColorForeground(dblue)
          .setColorBackground(dblue)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("THROTTLE")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("throttleValue")
          .setPosition(341,44)
          .setSize(114,64)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("thrustTitle")
          .setPosition(465,8)
          .setSize(114,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(white)
          .setColorForeground(dblue)
          .setColorBackground(dblue)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("THRUST")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("thrustValue")
          .setPosition(465,44)
          .setSize(114,64)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("currentTitle")
          .setPosition(589,8)
          .setSize(114,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(white)
          .setColorForeground(dblue)
          .setColorBackground(dblue)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("CURRENT")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("currentValue")
          .setPosition(589,44)
          .setSize(114,64)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("loopTitle")
          .setPosition(713,8)
          .setSize(114,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(white)
          .setColorForeground(dblue)
          .setColorBackground(dblue)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("LOOPRATE")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("loopValue")
          .setPosition(713,44)
          .setSize(114,64)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("refreshTitle")
          .setPosition(837,8)
          .setSize(114,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(white)
          .setColorForeground(dblue)
          .setColorBackground(dblue)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("FRAMERATE")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("refreshValue")
          .setPosition(837,44)
          .setSize(114,64)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addBang("zero")
          .setPosition(836,124)
          .setSize(116,48)
          .setFont(createFont("agency FB Bold",24,true))
          .getCaptionLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addBang("resetThrust")
          .setPosition(8,432)
          .setSize(214,46)
          .setFont(createFont("agency FB Bold",24,true))
          .setCaptionLabel("RESET THRUST")
          .getCaptionLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addBang("resetCurrent")
          .setPosition(8,642)
          .setSize(214,46)
          .setFont(createFont("agency FB Bold",24,true))
          .setCaptionLabel("RESET CURRENT")
          .getCaptionLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsThrustTitle")
          .setPosition(9,276)
          .setSize(212,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(white)
          .setColorForeground(dblue)
          .setColorBackground(dblue)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("THRUST")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsThrustMinTitle")
          .setPosition(11,312)
          .setSize(82,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("MIN")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsThrustAvgTitle")
          .setPosition(11,350)
          .setSize(82,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("AVG")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsThrustMaxTitle")
          .setPosition(11,388)
          .setSize(82,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("MAX")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsThrustMin")
          .setPosition(97,312)
          .setSize(122,36)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("min")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsThrustAvg")
          .setPosition(97,350)
          .setSize(122,36)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("avg")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsThrustMax")
          .setPosition(97,388)
          .setSize(122,36)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("max")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
 cp5.addTextfield("resultsCurrentTitle")
          .setPosition(9,486)
          .setSize(212,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(white)
          .setColorForeground(dblue)
          .setColorBackground(dblue)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("CURRENT")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsCurrentMinTitle")
          .setPosition(11,522)
          .setSize(82,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("MIN")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsCurrentAvgTitle")
          .setPosition(11,560)
          .setSize(82,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("AVG")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsCurrentMaxTitle")
          .setPosition(11,598)
          .setSize(82,36)
          .setFont(createFont("agency FB Bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("MAX")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsCurrentMin")
          .setPosition(97,522)
          .setSize(122,36)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("min")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsCurrentAvg")
          .setPosition(97,560)
          .setSize(122,36)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("avg")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("resultsCurrentMax")
          .setPosition(97,598)
          .setSize(122,36)
          .setFont(createFont("arial bold",24,true))
          .setColor(dblue)
          .setColorForeground(white)
          .setColorBackground(white)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setText("max")
          .setLock(true)
          .getValueLabel().align(ControlP5.CENTER,ControlP5.CENTER);
  cp5.addTextfield("serialData")
          .setPosition(0,height - 24)
          .setSize(width,24)
          .setFont(createFont("arial",12,false))
          .setColor(black)
          .setColorForeground(lgray)
          .setColorBackground(lgray)
          .setColorActive(dblue)
          .setCaptionLabel("")
          .setLock(true)
          .getValueLabel().getStyle().setPaddingLeft(5);
  thrustChart = cp5.addChart("thrustflow")
          .setPosition(230, 276)
          .setSize(722, 202)
          .setColorBackground(lgray)
          .setRange(-20, 2000)
          .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
          .setStrokeWeight(4)
          .setCaptionLabel("");
  currentChart = cp5.addChart("currentflow")
          .setPosition(230, 486)
          .setSize(722, 202)
          .setColorBackground(lgray)
          .setRange(-0.4, 40)
          .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
          .setStrokeWeight(4)
          .setCaptionLabel("");
  cp5.addScrollableList("dropdown")
          .setPosition(836, 180)
          .setSize(116, 192)
          .setBarHeight(24)
          .setItemHeight(24)
          .setCaptionLabel("SELECT COM PORT")
          .setColorForeground(vlblue)
          .setColorBackground(lblue)
          .setColorActive(vlblue)
          .setColorCaptionLabel(white)
          .setOpen(false) 
          .addItems(ports)
          .onClick(new CallbackListener() {
            public void controlEvent(CallbackEvent theEvent) {
              ports = Arrays.asList(Serial.list());
              cp5.get(ScrollableList.class, "dropdown").clear();
              cp5.get(ScrollableList.class, "dropdown").addItems(ports);
              println("Refresh");
            }
          });
  thrustChart.addDataSet("thrustData");
  thrustChart.setColors("thrustData", color(255,0,0));
  thrustChart.setData("thrustData", new float[940]);
  currentChart.addDataSet("currentData");
  currentChart.setColors("currentData", color(255,0,0));
  currentChart.setData("currentData", new float[940]);

  voltage = 0.0;
  throttle = 0.0;
  thrust = 0.0;
  minThrust = 0.0;
  avgThrust = 0.0;
  maxThrust = 0.0;
  current = 0.0;
  minCurrent = 0.0;
  avgCurrent = 0.0;
  maxCurrent = 0.0;
  time = 0.0;
  prevTime = 0.0;
  dTime = 0.0;
  idleThrottle = 0.0;
  runThrottle = 0.0;
  testStep = 0.0;
  testElapsed = 0.0;
  test = 0;
  prevTest = 0;
  cp5.get(Toggle.class,"armToggle").setBroadcast(true);
}

void draw () {
  ParseData();
  cp5.get(Textfield.class,"batteryValue").setText(nf(voltage,1,1) + " V");
  cp5.get(Textfield.class,"throttleValue").setText(nf(throttle,1,1) + " %");
  cp5.get(Textfield.class,"thrustValue").setText(nf(thrust,1,1) + " g");
  cp5.get(Textfield.class,"resultsThrustMin").setText(nf(minThrust,1,1) + " g");
  cp5.get(Textfield.class,"resultsThrustAvg").setText(nf(avgThrust,1,1) + " g");
  cp5.get(Textfield.class,"resultsThrustMax").setText(nf(maxThrust,1,1) + " g");
  cp5.get(Textfield.class,"currentValue").setText(nf(current,1,1) + " A");
  cp5.get(Textfield.class,"resultsCurrentMin").setText(nf(minCurrent,1,1) + " A");
  cp5.get(Textfield.class,"resultsCurrentAvg").setText(nf(avgCurrent,1,1) + " A");
  cp5.get(Textfield.class,"resultsCurrentMax").setText(nf(maxCurrent,1,1) + " A");
  cp5.get(Textfield.class,"loopValue").setText(nf((1.0 / dTime),1,1) + " Hz");
  cp5.get(Textfield.class,"refreshValue").setText(nf(round(frameRate)) + " FPS");
  cp5.get(Textfield.class,"serialData").setText("SERIAL DATA: " + dataStr);
  thrustChart.push("thrustData",thrust);
  currentChart.push("currentData",current);
  
  if (test == 1) {
    if (prevTest == 0) startTest();
    if (time != prevTime) logData();
  }
  else {
    if (prevTest == 1) {
      endTest();
      cp5.get(Toggle.class,"testToggle").setState(false);
      println("Test complete and logged");
    }
  }
  
  background(white);
  noStroke();
  fill(dgray);
  rect(0,0,width,116);
  fill(lgray);
  rect(0,116,width,152);
  fill(dblue);
  rect(8,276,214,150);
  rect(8,486,214,150);
  cp5.draw();
}

void serialEvent (Serial myPort) {
  dataStr = myPort.readStringUntil('\n');
}

void controlEvent(ControlEvent theEvent) {
  if(theEvent.isController()) {
    String name = theEvent.getController().getName();
    println("Control event from : " + name);
  }
}

// Example serial string: "Run Time(s): 25.687 / Loop Time(s): 0.01066 / Battery(V): 16.7 / Throttle(%): 0.00 / Thrust(g): 0.0 / Current(A): 0.0 / Test: 0"
void ParseData () {
  if (dataStr != null) {
    String[] paramArray, valArray, matchArray;
    paramArray = splitTokens(trim(dataStr),"/");
    for (int i = 0; i < paramArray.length; i++) {
      valArray = splitTokens(trim(paramArray[i]),":");
      if (valArray.length == 2) {
        matchArray = match(valArray[0], "Run Time");
        if (matchArray != null) {
          prevTime = time;
          time = float(trim(valArray[1]));
          continue;
        }
        matchArray = match(valArray[0], "Loop Time");
        if (matchArray != null) {
          dTime = float(trim(valArray[1]));
          continue;
        }
        matchArray = match(valArray[0], "Battery");
        if (matchArray != null) {
          voltage = float(trim(valArray[1]));
          continue;
        }
        matchArray = match(valArray[0], "Throttle");
        if (matchArray != null) {
          throttle = float(trim(valArray[1]));
          continue;
        }
        matchArray = match(valArray[0], "Thrust");
        if (matchArray != null) {
          thrust = float(trim(valArray[1]));
          if (thrust < minThrust) minThrust = thrust;
          if (thrust > maxThrust) {
            maxThrust = thrust;
            thrustChart.setRange((-(maxThrust * 1.1f) / 100.0f), (maxThrust * 1.1f));
          }
          continue;
        }
        matchArray = match(valArray[0], "Current");
        if (matchArray != null) {
          current = float(trim(valArray[1]));
          if (current < minCurrent) minCurrent = current;
          if (current > maxCurrent) {
            maxCurrent = current;
            currentChart.setRange((-(maxCurrent * 1.1f) / 100.0f), (maxCurrent * 1.1f));
          }
          continue;
        }
        matchArray = match(valArray[0], "Test");
        if (matchArray != null) {
          prevTest = test;
          test = int(trim(valArray[1]));
          continue;
        }
      }
    }
  }
}

public void startTest() {
  results = new Table();
  results.addColumn("TIME");
  results.addColumn("THROTTLE");
  results.addColumn("THRUST");
  results.addColumn("CURRENT");
  results.addColumn("VOLTAGE");
  time = 0.0;
}

public void logData() {
  TableRow newRow = results.addRow();
  newRow.setFloat("TIME",time);
  newRow.setFloat("THROTTLE",throttle);
  newRow.setFloat("THRUST",thrust);
  newRow.setFloat("CURRENT",current);
  newRow.setFloat("VOLTAGE",voltage);
}

public void endTest() {
  saveTable(results,"Results.csv");
}

public void disarm () {
  cp5.get(Toggle.class,"armToggle").setCaptionLabel("ARM");
  cp5.get(Toggle.class,"idleToggle").setBroadcast(false);
  cp5.get(Toggle.class,"runToggle").setBroadcast(false);
  cp5.get(Toggle.class,"testToggle").setBroadcast(false);
  cp5.get(Toggle.class,"idleToggle").setState(false);
  cp5.get(Toggle.class,"runToggle").setState(false);
  cp5.get(Toggle.class,"testToggle").setState(false);
  cp5.get(Toggle.class,"idleToggle").setLock(true);
  cp5.get(Toggle.class,"runToggle").setLock(true);
  cp5.get(Toggle.class,"testToggle").setLock(true);
  cp5.get(Toggle.class,"idleToggle").setColorBackground(bgray);
  cp5.get(Toggle.class,"runToggle").setColorBackground(bgray);
  cp5.get(Toggle.class,"testToggle").setColorBackground(bgray);
  cp5.get(Slider.class,"idleSlider").setLock(false);
  cp5.get(Slider.class,"runSlider").setLock(false);
  cp5.get(Slider.class,"testSlider").setLock(false);
  myPort.write("p0.0\n");
  println("SENT: p0.0");
}

public void arm() {
  cp5.get(Toggle.class,"armToggle").setCaptionLabel("STOP");
  cp5.get(Toggle.class,"idleToggle").setLock(false);
  cp5.get(Toggle.class,"runToggle").setLock(false);
  cp5.get(Toggle.class,"testToggle").setLock(false);
  cp5.get(Toggle.class,"idleToggle").setColorBackground(lblue); //<>//
  cp5.get(Toggle.class,"runToggle").setColorBackground(lblue);
  cp5.get(Toggle.class,"testToggle").setColorBackground(lblue);
  cp5.get(Toggle.class,"idleToggle").setBroadcast(true);
  cp5.get(Toggle.class,"runToggle").setBroadcast(true);
  cp5.get(Toggle.class,"testToggle").setBroadcast(true);
}

public void zero() {
  myPort.write("z\n");
  println("Zero SENT: z");
}

public void resetThrust() {
  minThrust = thrust;
  avgThrust = thrust;
  maxThrust = thrust;
  thrustChart.setData("thrustData", new float[940]);
}

public void resetCurrent() {
  minCurrent = current;
  avgCurrent = current;
  maxCurrent = current;
  currentChart.setData("currentData", new float[940]);
}

void armToggle(boolean state) {
  if (state) arm();
  else disarm();
}

public void idleToggle(boolean state) {
  if (state) {
    cp5.get(Slider.class,"idleSlider").setLock(true);
    deactivateToggle("runToggle");
    deactivateToggle("testToggle");
    myPort.write("p" + cp5.get(Slider.class,"idleSlider").getValue() + "\n");
    println("Idle SENT: p" + cp5.get(Slider.class,"idleSlider").getValue());
  }
  else {
    cp5.get(Slider.class,"idleSlider").setLock(false);
    myPort.write("p0.0\n");
    println("Idle SENT: p0.0");
  }
}

public void runToggle(boolean state) {
  if (state) {
    cp5.get(Slider.class,"runSlider").setLock(true);
    deactivateToggle("idleToggle");
    deactivateToggle("testToggle");
    myPort.write("p" + cp5.get(Slider.class,"runSlider").getValue() + "\n");
    println("Run SENT: p" + cp5.get(Slider.class,"runSlider").getValue());
  }
  else {
    cp5.get(Slider.class,"runSlider").setLock(false);
    myPort.write("p0.0\n");
    println("Run SENT: p0.0");
  }
}

public void testToggle(boolean state) {
  if (state) {
    cp5.get(Slider.class,"testSlider").setLock(true);
    deactivateToggle("idleToggle");
    deactivateToggle("runToggle");
    idleThrottle = cp5.get(Slider.class,"idleSlider").getValue();
    runThrottle = cp5.get(Slider.class,"runSlider").getValue();
    testStep = cp5.get(Slider.class,"testSlider").getValue();
    testElapsed = 0.0;
    myPort.write("i" + idleThrottle + "\n");
    println("Test SENT: i" + idleThrottle);
    myPort.write("r" + runThrottle + "\n");
    println("Test SENT: r" + runThrottle);
    myPort.write("s" + testStep + "\n");
    println("Test SENT: s" + testStep);
  }
  else {
    cp5.get(Slider.class,"testSlider").setLock(false);
    myPort.write("p0.0\n");
    println("Test SENT: p0.0");
  }
}

void dropdown(int n) {
  if (myPort != null) {
    myPort.stop();
  }
  myPort = new Serial(this, cp5.get(ScrollableList.class, "dropdown").getItem(n).get("name").toString(), 250000);  // and use the correct index number in Serial.list()[].
  myPort.bufferUntil('\n');                             // A serialEvent() is generated when a newline character is received :
  println("Opening: " + cp5.get(ScrollableList.class, "dropdown").getItem(n).get("name").toString());
}

public void deactivateToggle(String name) {
  cp5.get(Toggle.class,name).setBroadcast(false);
  cp5.get(Toggle.class,name).setState(false);
  cp5.get(Toggle.class,name).setBroadcast(true);
}
