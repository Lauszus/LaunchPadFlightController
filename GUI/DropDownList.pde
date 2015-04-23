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

ControlP5 controlP5;
DropdownList dropdownList; // Define the variable ports as a Dropdownlist

final String portnameFile = "SerialPort.txt"; // Name of file for last connected serial port

boolean connectedSerial;
int portNumber = -1; // The dropdown list will return a float value, which we will connvert into an int. We will use this int for that.

void initDropdownlist() {
  dropdownList = controlP5.addDropdownList("SerialPort"); // Make a dropdown list with all serial ports

  dropdownList.setPosition(10, 20);
  dropdownList.setSize(210, 200);
  dropdownList.setCaptionLabel("Select serial port"); // Set the lable of the bar when nothing is selected

  dropdownList.setBackgroundColor(color(200)); // Set the background color of the line between values
  dropdownList.setItemHeight(20); // Set the height of each item when the list is opened
  dropdownList.setBarHeight(15); // Set the height of the bar itself

  dropdownList.getCaptionLabel().getStyle().marginTop = 3; // Set the top margin of the lable
  dropdownList.getCaptionLabel().getStyle().marginLeft = 3; // Set the left margin of the lable

  dropdownList.setColorBackground(color(0));
  dropdownList.setColorActive(color(255, 128));

  BufferedReader reader = createReader(portnameFile);
  String line = null;
  try {
    line = reader.readLine();
    println("Previous serial port: " + line);
  } catch (IOException e) {
    e.printStackTrace();
  }

  // Now add the ports to the list, we use a for loop for that
  for (int i = 0; i < Serial.list().length; i++) {
    if (Serial.list()[i].indexOf("/dev/cu.") != -1)
      continue; // Do not display /dev/cu.* devices
    dropdownList.addItem(Serial.list()[i], i); // This is the line doing the actual adding of items, we use the current loop we are in to determine what place in the char array to access and what item number to add it as
    if (line != null && line.equals(Serial.list()[i]))
      dropdownList.setValue(i); // Automatically select the last selected serial port
  }

  addMouseWheelListener(new MouseWheelListener() { // Add a mousewheel listener to scroll the dropdown list
    public void mouseWheelMoved(MouseWheelEvent mwe) {
      dropdownList.scroll(mwe.getWheelRotation() > 0 ? 1 : 0); // Scroll the dropdownlist using the mousewheel
    }
  });

  controlP5.addButton("connect")
           .setPosition(225, 3)
           .setSize(41, 15);

  controlP5.addButton("disconnect")
           .setPosition(271, 3)
           .setSize(52, 15);

  controlP5.addButton("reset")
           .setPosition(328, 3)
           .setSize(52, 15)
           .setCaptionLabel("Reset yaw");
}

void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) {
    if (theEvent.getGroup().getName() == dropdownList.getName())
      portNumber = int(theEvent.getGroup().getValue()); // Since the list returns a float, we need to convert it to an int. For that we us the int() function
  }
}

void connect() {
  disconnect(); // Disconnect any existing connection
  if (portNumber != -1) { // Check if com port and baudrate is set and if there is not already a connection established
    println("ConnectSerial");
    dropdownList.close();
    try {
      serial = new Serial(this, Serial.list()[portNumber], 115200);
    } catch (Exception e) {
      println("Couldn't open serial port");
      e.printStackTrace();
    }
    if (serial != null) {
      SaveSerialPort(Serial.list()[portNumber]);
      serial.bufferUntil('\n');
      connectedSerial = true;
    }
  } else if (portNumber == -1)
    println("Select COM Port first!");
  else if (connectedSerial)
    println("Already connected to a port!");
}

void disconnect() {
  if (!connectedSerial)
    return;
  try {
    serial.stop();
    serial.clear(); // Empty the buffer
    connectedSerial = false;
    println("DisconnectSerial");
  } catch (Exception e) {
    //e.printStackTrace();
    println("Couldn't disconnect serial port");
  }
}

void SaveSerialPort(String port) {
  PrintWriter output = createWriter(portnameFile);
  output.print(port); // Write the comport to the file
  output.flush(); // Writes the remaining data to the file
  output.close(); // Finishes the file
}

