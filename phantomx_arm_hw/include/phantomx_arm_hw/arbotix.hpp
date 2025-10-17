#ifndef ARBOTIX_HPP_
#define ARBOTIX_HPP_

#include <math.h>
#include <string>

#include "lx_serial.hpp"

// Low-level interaction with Arbotix-M board.
// Only sync read and write are supported.
class Arbotix
{
  protected:
    LxSerial port_;

  public:
    Arbotix() { }
    ~Arbotix() { }

    // Open serial port.
    bool open(std::string &port_name);

    // Get serial port status.
    bool is_port_open();

    // Close serial port.
    bool close();

    // Read control table entries [addr, addr+n_bytes> from ids. buf is of size n_ids*n_bytes.
    bool read(unsigned char *ids, unsigned char n_ids, unsigned char addr, unsigned char n_bytes, unsigned char *buf);

    // Write control table entries [addr, addr+n_bytes> to ids. buf is of size n_ids*n_bytes.
    bool write(unsigned char *ids, unsigned char n_ids, unsigned char addr, unsigned char n_bytes, unsigned char *buf);

    // Convert Dynamixel servo position to radians.
    double pos2rad(int pos)
    {
      return (pos-512)/1023. * (5/3. * M_PI);
    }

    // Convert radians to Dynamixel servo position.
    int rad2pos(double angle)
    {
      return std::min(std::max(int(angle / (5/3.*M_PI) * 1023 + 512), 0), 1023);
    }

    // Convert Dynamixel servo speed to radians per second.
    double speed2rads(int speed)
    {
      return (speed&1023) * (0.111 / 60 * 2 * M_PI) * ((speed&1024)?-1:1);
    }

    // Convert radians per second to Dynamixel servo speed.
    int rads2speed(double rads)
    {
      return std::min(std::max(int(rads / (0.111 / 60 * 2 * M_PI)), 1), 1023);
    }

    // Calculate Dynamixel packet checksum.
    unsigned char checksum(unsigned char *buf, int n)
    {
      unsigned char sum=0;
      for (size_t ii=0; ii != n; ++ii)
        sum += buf[ii];
      return 255-sum;
    }
};

#endif // ARBOTIX_HPP_
