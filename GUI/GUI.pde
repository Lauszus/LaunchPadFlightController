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
    float radRoll = radians(roll);
    float radPitch = radians(pitch);
    float radYaw = radians(yaw);
    rotateX(radPitch * cos(radYaw) - radRoll * sin(radYaw) + HALF_PI);
    rotateY(radRoll * cos(radYaw) + radPitch * sin(radYaw));
    rotateZ(radYaw * cos(radPitch) + radRoll * sin(radPitch));
    box(200, 200, 30);
    popMatrix();

    System.out.printf("%.02f, %.02f, %.02f\n", roll, pitch, yaw);
  }
}

byte[] buffer;
boolean append;

void serialEvent (Serial serial) {
  if (append)
    buffer = concat(buffer, serial.readBytes());
  else
    buffer = serial.readBytes();
  int[] data = new int[buffer.length];
  for (int i = 0; i < data.length; i++)
    data[i] = buffer[i] & 0xFF; // Cast to unsigned value
  //println(data);

  if (new String(buffer).startsWith(responseHeader)) {
    int cmd = data[responseHeader.length()];
    int length = data[responseHeader.length() + 1];
    if (length != (data.length -  responseHeader.length() - 5)) {
      append = true; // If it's not the correct length, then the rest will come in the next package
      return;
    }
    int input[] = new int[length];
    int i;
    for (i = 0; i < length; i++)
      input[i] = data[i + responseHeader.length() + 2];
    int checksum = data[i + responseHeader.length() + 2];

    if (checksum == (cmd ^ length ^ getChecksum(input))) {
      switch(cmd) {
        case SEND_ANGLES:
          int rollInt = input[0] | ((byte) input[1] << 8); // This can be negative as well
          int pitchInt = input[2] | ((byte) input[3] << 8); // This can be negative as well
          int yawInt = input[4] | (input[5] << 8); // This is always positive
          //println(rollInt + " " + pitchInt + " " + yawInt);

          roll = (float) rollInt / 100.0f; // Convert values to floating point numbers
          pitch = (float) pitchInt / 100.0f; // Convert values to floating point numbers
          yaw = (float) yawInt / 100.0f; // Convert values to floating point numbers

          if (resetYaw) {
            resetYaw  = false;
            yawStart = yaw;
          }
          yaw -= yawStart; // Subtract starting angle
          if (yaw < 0) // Convert heading range to 0-360
            yaw += 360;
          break;
        default:
          println("Unknown command");
          break;
      }
    } else
      println("Checksum error!");
  } else {
    println("Wrong header!");
    println(new String(buffer));
  }

  serial.clear(); // Clear buffer
  drawValues = true; // Draw the plane
  append = false;
}

void reset() {
  resetYaw = true; // Reset yaw position
}

void keyPressed() {
  if (key == 'r')
    reset();
  else if (key == ENTER || key == 'c')
    connect(); // Connect to serial port
  else if (key == ESC || key == 'd' ) {
    disconnect(); // Disconnect serial connection
    key = 0; // Disable Processing from quiting when pressing ESC
  }
}

