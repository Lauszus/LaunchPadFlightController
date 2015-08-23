/* Copyright (C) 2015 Kristian Sloth Lauszus. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Sloth Lauszus
 Web      :  http://www.lauszus.com
 e-mail   :  lauszus@gmail.com
*/

String commandHeader = "$S>"; // Standard command header
String responseHeader = "$S<"; // Standard response header

static final byte SEND_ANGLES = 8;

void sendCommand(byte output[]) {
  if (connectedSerial) {
    serial.write(commandHeader);
    serial.write(output);
    serial.write(getChecksum(output));
  } else
    println("Establish a serial connection first!");
}

void sendAngles(boolean enable) {
  byte output[] = {
    SEND_ANGLES, // Cmd
    1, // Length
    (byte) (enable ? 1 : 0),
  };
  sendCommand(output); // Send output
}

byte getChecksum(byte data[]) {
  byte checksum = 0;
  for (byte val : data)
    checksum ^= val;
  return checksum;
}

int getChecksum(int data[]) {
  int checksum = 0;
  for (int val : data)
    checksum ^= val;
  return checksum;
}
