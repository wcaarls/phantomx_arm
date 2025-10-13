//============================================================================
// Adapted from: LxSerial.cpp
// Author      : Eelko van Breda,www.dbl.tudelft.nl
// Version     : 0.1
// Copyright   : Copyright (c) 2008 LGPL
// Description : serial communicatin class linux
//============================================================================

#include <stdio.h>

#include <phantomx_arm_hw/lx_serial.hpp>

/*  constructor */
LxSerial::LxSerial()
{
    hPort = INVALID_DEVICE_HANDLE;
}

/* return name of the port that was opened */
std::string& LxSerial::get_name()
{
    return s_port_name;
}

/* open port */
bool LxSerial::open(const std::string& portname)
{
    // Open port
    hPort = ::open(portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);                   // open the serial device

    if (hPort < 0) {                                        // check if port opened correctly
        perror(" Could not open serial port, aborting");
        return false;
    }

    tcgetattr(hPort, &options);                                     // get the current termios (com port options struct) from the kernel
    tcgetattr(hPort, &old_options);                                 // get the current termios copy to restore on close

    cfsetispeed(&options, B115200);                                 // set incomming baudrate to standard 115200, this can be changed with the function set_speed()
    cfsetospeed(&options, B115200);                                 // set outgoing baudrate to standard 115200

    options.c_cflag |= (CLOCAL|CREAD|CS8);                              // CREAD = enanble receiver
    options.c_cflag &= ~(CRTSCTS|PARENB|CSTOPB);                            // Disable HW flow control

    options.c_lflag &= ~(ECHO|ECHONL|ECHOE|ICANON|ISIG|IEXTEN);
    options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON|IXOFF);
    options.c_oflag &= ~(OPOST);

    options.c_cc[VMIN]     = 0;                                     // VMIN Minimum number of characters to read
    options.c_cc[VTIME]    = WAIT_FOR_DATA_DSEC;                            // Time to wait for data (tenths of seconds)

    if (tcsetattr(hPort,TCSANOW, &options)!=0){                         // Set the new options for the port now
        perror("Error: Could not set serial port settings");
        return false;
    }

    usleep(100);                                            // additional wait for correct functionality
    tcflush(hPort, TCIOFLUSH);                                  // flush terminal data

    // Save port name
    s_port_name = portname;

    return true;
}

bool LxSerial::is_open()
{
    return hPort >= 0;
}

bool LxSerial::close()
{
    if (hPort==INVALID_DEVICE_HANDLE)
        return true;

    if (tcsetattr(hPort,TCSANOW, &old_options)!=0) {                        // restore the old port settings
        perror("Warning: Could not restore serial port settings.");
    }

    if(::close(hPort) == -1) {                                    //close serial port
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
    printf("read  ");
    for (int i=0;i<nBytesRead;i++)
        { printf("%02X ",buffer[i]); }
    printf("(%d)\n",nBytesRead);
    #endif

    return nBytesRead;
}

int LxSerial::read(unsigned char* buffer, int numBytes, int seconds, int microseconds)
{
    // Init time variables (they are decreased by wait_for_input)
    int s = seconds;
    int us = microseconds;
    int nBytesRead = 0;
    while (nBytesRead < numBytes)
    {
        if( wait_for_input( &s, &us) )
        {
            int partialRead = ::read(hPort, buffer + nBytesRead, numBytes - nBytesRead);  // Read data
            nBytesRead += partialRead;
        }
        else
        {
            #ifdef __DBG__
            printf("Read Timeout... \n");
            #endif
            return nBytesRead;
        }
    }

    #ifdef __DBG__
    printf("read  ");
    for (int i=0;i<nBytesRead;i++)
        { printf("%02X ",buffer[i]); }
    printf("(%d)\n",nBytesRead);
    #endif

    return nBytesRead;
}

bool LxSerial::wait_for_input(int *seconds, int *microseconds)
{
    fd_set readset;
    timeval timeout;
    timeout.tv_sec = *seconds;                                  // seconds
    timeout.tv_usec = *microseconds;                                // microseconds
    FD_ZERO(&readset);                                          // clear file discriptor
    FD_SET(hPort, &readset);                                    // set filediscripter for port
    int res = select(hPort+1, &readset, NULL, NULL, &timeout);  // wait till readable data is in the buffer
    *seconds = timeout.tv_sec;
    *microseconds = timeout.tv_usec;
    return res == 1;
}

int LxSerial::write(unsigned char* buffer, int numBytes)
{
    int msc = TIOCM_RTS;
    int numBytesWritten = ::write(hPort, buffer, numBytes);                       // write data

    if (numBytes != numBytesWritten){
        perror("Error while writing to serial port");
        assert(numBytes == numBytesWritten);
    }

    #ifdef __DBG__
    printf("write ");
    for (int i=0;i<numBytes;i++)
        { printf("%02X ",buffer[i]); }
    printf("(%d)",numBytesWritten);
    printf("\n");
    #endif

    tcdrain(hPort);                                         // Wait till all the data in the buffer is transmitted

    return numBytesWritten;
}

void LxSerial::flush()
{
    tcflush(hPort, TCIOFLUSH);                                  // flush data buffer
}

LxSerial::~LxSerial()
{
    // Warn when you forgot to close the port before destruction.
    // We CANNOT call port_close() here, because port_close is a virtual function
    // and virtual functions cannot be called in constructors and destructors.
    if (hPort != INVALID_DEVICE_HANDLE)
        printf("[LxSerial] Warning: you didn't call port_close before calling the destructor.\n");
}
