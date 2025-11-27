// Adapted from LxSerial.cpp by Eelko van Breda, www.dbl.tudelft.nl
// Copyright (c) 2008 LGPL

#include <stdio.h>
#include <iomanip>

#include <phantomx_arm_hw/lx_serial.hpp>
#include <rclcpp/rclcpp.hpp>

#define ROS_DEBUG_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("PhantomXArmHardware"), x)

//#define __DBG__

LxSerial::LxSerial()
{
  hPort = INVALID_DEVICE_HANDLE;
}

LxSerial::~LxSerial()
{
  if (hPort != INVALID_DEVICE_HANDLE)
    close();
}

bool LxSerial::open(const std::string& portname)
{
  // Open port
  hPort = ::open(portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (hPort < 0) {
    perror(" Could not open serial port, aborting");
    return false;
  }

  // Get the current termios (com port options struct) from the kernel.
  tcgetattr(hPort, &options);

  // Get the current termios copy to restore on close.
  tcgetattr(hPort, &old_options);

  // set incoming and outgoing baudrate to 115200.
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // 8 data bits, do not change owner of port
  options.c_cflag |= (CLOCAL|CREAD|CS8);

  // No hardware flow control, no parity, 1 stop bit.
  options.c_cflag &= ~(CRTSCTS|PARENB|CSTOPB);

  // No echo, raw input, no signals or extended functions
  options.c_lflag &= ~(ECHO|ECHONL|ECHOE|ICANON|ISIG|IEXTEN);

  // NO break signal signal. Leave bits alone. No software flow control.
  options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON|IXOFF);

  // Raw output.
  options.c_oflag &= ~(OPOST);

  // Minimum number of characters to read.
  options.c_cc[VMIN] = 0;

  // Time to wait for data (tenths of seconds).
  options.c_cc[VTIME] = WAIT_FOR_DATA_DSEC;

  // Set the new port options.
  if (tcsetattr(hPort,TCSANOW, &options)!=0) {
    perror("Error: Could not set serial port settings");
    return false;
  }

  // Wait for things to settle and flush buffers.
  usleep(100);
  tcflush(hPort, TCIOFLUSH);                                  

  // Save port name.
  s_port_name = portname;

  return true;
}

bool LxSerial::is_open()
{
  return hPort >= 0;
}

std::string& LxSerial::get_name()
{
  return s_port_name;
}

bool LxSerial::close()
{
  if (hPort==INVALID_DEVICE_HANDLE)
    return true;

  // Restore the old port settings.
  if (tcsetattr(hPort,TCSANOW, &old_options)!=0) {                        
    perror("Warning: Could not restore serial port settings.");
  }

  // Close serial port.
  if(::close(hPort) == -1) {                                    
    perror("Error: Could not close serial port.");
    return false;
  }

  hPort = INVALID_DEVICE_HANDLE;

  return true;
}

int LxSerial::read(unsigned char* buffer, int numBytes) const
{
  int nBytesRead = ::read(hPort, buffer, numBytes);

  #ifdef __DBG__
    ROS_DEBUG_STREAM("read " << buf2str(buffer, nBytesRead));
  #endif

  return nBytesRead;
}

int LxSerial::read(unsigned char* buffer, int numBytes, int seconds, int microseconds)
{
  // Init time variables (they are decreased by wait_for_input).
  int s = seconds;
  int us = microseconds;
  int nBytesRead = 0;

  while (nBytesRead < numBytes)
  {
    if(wait_for_input(&s, &us))
    {
      // Read available data.
      int partialRead = ::read(hPort, buffer + nBytesRead, numBytes - nBytesRead);  
      nBytesRead += partialRead;
    }
    else
    {
      #ifdef __DBG__
        ROS_DEBUG_STREAM("Read Timeout...");
      #endif

      break;
    }
  }

  #ifdef __DBG__
    ROS_DEBUG_STREAM("read " << buf2str(buffer, nBytesRead));
  #endif

  return nBytesRead;
}

bool LxSerial::wait_for_input(int *seconds, int *microseconds)
{
  fd_set readset;
  timeval timeout;
  timeout.tv_sec = *seconds;
  timeout.tv_usec = *microseconds;

  // Set file discriptor only for our port.
  FD_ZERO(&readset);                                          
  FD_SET(hPort, &readset);                                    

  // Wait until readable data is in the buffer.
  int res = select(hPort+1, &readset, NULL, NULL, &timeout);

  // Update timeout based on how long we waited.
  *seconds = timeout.tv_sec;
  *microseconds = timeout.tv_usec;

  return res == 1;
}

int LxSerial::write(unsigned char* buffer, int numBytes)
{
  int numBytesWritten = ::write(hPort, buffer, numBytes);

  if (numBytes != numBytesWritten){
    perror("Error while writing to serial port");
    assert(numBytes == numBytesWritten);
  }

  #ifdef __DBG__
    ROS_DEBUG_STREAM("write " << buf2str(buffer, numBytesWritten));
  #endif

  // Wait until all the data in the buffer has been transmitted.
  tcdrain(hPort);                                         

  return numBytesWritten;
}

void LxSerial::flush()
{
  // Flush data buffers.
  tcflush(hPort, TCIOFLUSH);                                  
}

std::string LxSerial::buf2str(unsigned char *buffer, int numBytes) const
{
  std::ostringstream oss;
  for (int i=0;i<numBytes;i++)
    oss << std::setw(2) << std::setfill('0') << std::hex << (int)buffer[i] << " ";
  oss << "(" << std::dec << numBytes << ")";
  
  return oss.str();
}
