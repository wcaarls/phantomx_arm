// Adapted from LxSerial.cpp by Eelko van Breda, www.dbl.tudelft.nl
// Copyright (c) 2008 LGPL

#ifndef LXSERIAL_HPP_
#define LXSERIAL_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#define INVALID_DEVICE_HANDLE -1
#define WAIT_FOR_DATA_DSEC     5

// Linux serial port driver.
class LxSerial
{
  protected:
    int             hPort;                                                          // file handle to the port
    std::string     s_port_name;                                                    // name of the port that was opened
    termios         options, old_options;                                           //
    bool            wait_for_input(int *seconds, int *microseconds);                // private member function to wait for port. the time variables are modified after return to reflect the time not slept

  public:
    LxSerial();
    ~LxSerial();
    
    // Open serial port.
    bool open(const std::string& portname); 

    // Get serial port status.
    bool is_open();

    // Get port name.
    std::string& get_name();

    // Close serial port.
    bool close();

    // Read currently available data from serial port.
    int read(unsigned char* buffer, int numBytes) const;

    // Wait for and read specified amount of data from serial port.
    int read(unsigned char* buffer, int numBytes, int seconds, int microseconds);

    // Write data to serial port.
    int write(unsigned char* buffer, int numBytes);

    // Flush input and output buffers.
    void flush();
};

#endif // LXSERIAL_HPP_
