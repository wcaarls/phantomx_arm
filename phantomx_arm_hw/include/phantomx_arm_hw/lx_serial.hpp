//============================================================================
// Adapted from: LxSerial.cpp
// Author      : Eelko van Breda,www.dbl.tudelft.nl
// Version     : 0.1
// Copyright   : Copyright (c) 2008 LGPL
// Description : serial communicatin class linux
//============================================================================

#ifndef LXSERIAL_H_
#define LXSERIAL_H_
//#define __DBG__

#include <fcntl.h>                                                              /* fileio */
#include <termios.h>                                                            /* terminal i/o system, talks to /dev/tty* ports  */
#include <unistd.h>                                                             /* Read function */
#include <sys/ioctl.h>                                                          /* ioctl function */
#include <iostream>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#define INVALID_DEVICE_HANDLE       -1

class LxSerial
{
    public:
        /*return values*/
        static const int READ_ERROR         = -1;
        static const int WAIT_FOR_DATA_DSEC =  5;

    protected:
        int             hPort;                                                          // file handle to the port
        std::string     s_port_name;                                                    // name of the port that was opened
        termios         options, old_options;                                           //
        bool            wait_for_input(int *seconds, int *microseconds);                // private member function to wait for port. the time variables are modified after return to reflect the time not slept

    public:
                        LxSerial();
        virtual         ~LxSerial();
        virtual bool    open(const std::string& portname);   // open serial port. If overridden, make sure you set s_port_name!!
        virtual bool    is_open();
        std::string&    get_name();
        virtual bool    close();
        virtual int     read(unsigned char* buffer, int numBytes) const;
        virtual int     read(unsigned char* buffer, int numBytes, int seconds, int microseconds);
        virtual int     write(unsigned char* buffer, int numBytes);
        virtual void    flush();                                                 // flush input and output buffers
};

#endif /*LXSERIAL_H_*/
