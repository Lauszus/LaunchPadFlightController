/* Copyright (C) 2015 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
*/

import processing.serial.*;
import controlP5.*;
import java.awt.event.*;
import java.awt.Image;

Serial serial;

float roll, pitch, yaw;
float yawStart;

boolean resetYaw = true;
boolean drawValues  = false;

final int width = 600;
final int height = 600;

void setup() {  
  /*frame.setTitle("Balanduino Processing App");
   frame.setIconImage((Image) loadImage("data/logo.png").getNative());*/

  controlP5 = new ControlP5(this);
  size(width, height, P3D);

  initDropdownlist();
}

void draw() {
  if (!connectedSerial || drawValues) {
    background(250);
    lights();
  }

  if (drawValues) { // Draw plane
    drawValues = false;

    pushMatrix();
    translate(width/2, height/2, 0);
    rotateX(radians(pitch) + HALF_PI); // Add PI/2, so it sits flat by default
    rotateY(radians(roll));
    rotateZ(radians(yaw));
    box(200, 200, 30);
    popMatrix();

    System.out.printf("%.02f, %.02f, %.02f\n", roll, pitch, yaw);
  }
}

void serialEvent (Serial serial) {
  String[] input = trim(split(serial.readString(), '\t'));
  if (input.length != 6) {
    println("Wrong length: " + input.length);
    return;
  }

  roll = float(input[1]); // Store values
  pitch = float(input[3]);
  yaw = float(input[5]);

  if (resetYaw) {
    resetYaw  = false;
    yawStart = yaw;
  }

  yaw -= yawStart; // Subtract starting angle

  serial.clear(); // Clear buffer
  drawValues = true; // Draw the plane
}

void reset() {
  resetYaw = true; // Reset yaw position
}

void keyPressed() {
  if (key == 'r')
    reset();
  else if (key == ENTER || key == 'c')
    connect(); // Connect to serial port
  else if (key == 'd')
    disconnect(); // Disconnect serial connection
}

