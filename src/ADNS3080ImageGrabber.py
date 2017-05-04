#!/usr/bin/env python
#
# Copyright DIY Drone: https://github.com/diydrones/ardupilot/tree/5ddbcc296dd6dd9ac9ed6316ac3134c736ae8a78/libraries/AP_OpticalFlow/examples/ADNS3080ImageGrabber
# File: ADNS3080ImageGrabber.py

import string
from serial import Serial, SerialException
from Tkinter import Tk, Frame, StringVar, Button, Canvas, OptionMenu
from threading import Timer

import serial.tools.list_ports


def serial_ports():
    devices = []
    for port in list(serial.tools.list_ports.comports()):
        if port.hwid != 'n/a':  # Check if it is a physical port
            devices.append(port.device)
    if devices:
        print('Serial ports:')
        for device in devices:
            print('  ' + device)
    else:
        print('No serial ports found')
    return devices


class App:
    grid_size = 15
    num_pixels = 30
    image_started = False
    image_current_row = 0
    ser = None
    pixel_dictionary = {}
    t = None

    def __init__(self, master):
        # Set main window's title
        master.title("ADNS3080ImageGrabber")

        frame = Frame(master)
        frame.grid(row=0, column=0)

        self.comPortStr = StringVar()
        ports = serial_ports()
        if not ports:
            ports = ['No serial ports found']
        self.comPortStr.set(ports[0])  # Set first port as default

        comports = apply(OptionMenu, (frame, self.comPortStr) + tuple(ports))
        comports.grid(row=0, column=0)

        self.baudrateStr = StringVar()
        self.baudrateStr.set('115200')

        baudrates = apply(OptionMenu, (frame, self.baudrateStr) + tuple(Serial.BAUDRATES))
        baudrates.grid(row=0, column=1)

        button = Button(frame, text="Open", command=self.open_serial)
        button.grid(row=0, column=2)

        button = Button(frame, text="Close", command=self.close)
        button.grid(row=0, column=3)

        # self.entryStr = StringVar()
        # self.entry = Entry(frame, textvariable=self.entryStr)
        # self.entry.grid(row=0, column=3)
        # self.entry.delete(0, END)
        # self.entry.insert(0, "I")
        #
        # self.send_button = Button(frame, text="Send", command=self.send_to_serial)
        # self.send_button.grid(row=0, column=4)

        self.canvas = Canvas(master, width=self.grid_size*self.num_pixels, height=self.grid_size*self.num_pixels)
        self.canvas.grid(row=1)

    def __del__(self):
        self.close()

    def close(self):
        self.stop_read_loop()
        self.close_serial()

    def close_serial(self):
        if self.ser and self.ser.isOpen():
            try:
                self.ser.close()
                print("Closed serial port")
            except (IOError, SerialException):
                pass  # Do nothing

    def open_serial(self):
        # Close the serial port
        self.close_serial()
        # Open the serial port
        try:
            self.ser = Serial(port=self.comPortStr.get(), baudrate=self.baudrateStr.get(), timeout=0.1)
            print("Serial port '" + self.comPortStr.get() + "' opened at " + self.baudrateStr.get())
            # Start attempts to read from serial port
            self.read_loop()
        except SerialException:
            print("Failed to open serial port '" + self.comPortStr.get() + "'")

    # def send_to_serial(self):
    #     if self.ser.isOpen():
    #         self.ser.write(self.entryStr.get())
    #         print("Sent '" + self.entryStr.get() + "' to " + self.ser.portstr)
    #     else:
    #         print("Serial port not open!")

    def read_loop(self):
        if self.t:
            self.t.cancel()
        # print("reading")
        if self.ser.isOpen():
            try:
                self.read_from_serial()
            except IOError:
                self.close()

        self.t = Timer(0.0, self.read_loop)
        self.t.start()

    def stop_read_loop(self):
        try:
            self.t.cancel()
        except AttributeError:
            print("Failed to cancel timer")

    def read_from_serial(self):
        while self.ser and self.ser.isOpen() and self.ser.inWaiting() > 0:
            line_processed = False
            line = self.ser.readline()

            # Process the line read
            if line.find("-------------------------") == 0:
                line_processed = True
                self.image_started = False
                self.image_current_row = 0

            if self.image_started:
                if self.image_current_row >= self.num_pixels:
                    self.image_started = False
                else:
                    words = string.split(line, ",")
                    if len(words) >= self.num_pixels:
                        line_processed = True
                        x = 0
                        for v in words:
                            try:
                                colour = int(v)
                            except ValueError:
                                colour = 0
                            # self.display_pixel(x,self.image_current_row,colour)
                            self.display_pixel(self.num_pixels-1-self.image_current_row, self.num_pixels-1-x, colour)
                            x += 1
                        self.image_current_row += 1
                    else:
                        print("Line " + str(self.image_current_row) + "incomplete (" + str(len(words)) + " of "
                              + str(self.num_pixels) + "), ignoring")
                        # print("Bad line: " + line);

            if line.find("image data") >= 0:
                line_processed = True
                self.image_started = True
                self.image_current_row = 0
                # Clear canvas
                # self.canvas.delete(ALL)  # Remove all items

            # Display the line if we couldn't understand it
            if not line_processed:
                print(line)

    def display_default_image(self):
        # Display the grid
        for x in range(0, self.num_pixels-1):
            for y in range(0, self.num_pixels-1):
                colour = x * y / 3.53
                self.display_pixel(x, y, colour)

    def display_pixel(self, x, y, colour):
        if 0 <= x < self.num_pixels and 0 <= y < self.num_pixels:
            # Find the old pixel if it exists and delete it
            if x+y*self.num_pixels in self.pixel_dictionary:
                old_pixel = self.pixel_dictionary[x+y*self.num_pixels]
                self.canvas.delete(old_pixel)
                del old_pixel

            fill_colour = "#%02x%02x%02x" % (colour, colour, colour)
            # Draw a new pixel and add to pixel_array
            new_pixel = self.canvas.create_rectangle(x*self.grid_size, y*self.grid_size, (x+1)*self.grid_size,
                                                     (y+1)*self.grid_size, fill=fill_colour)
            self.pixel_dictionary[x+y*self.num_pixels] = new_pixel


root = Tk()
# root.withdraw()

# Create main display
app = App(root)
app.display_default_image()

print("Entering main loop!")

root.mainloop()

app.stop_read_loop()

print("Exiting")
