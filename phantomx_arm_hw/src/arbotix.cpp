#include <errno.h>

#include <rclcpp/rclcpp.hpp>
#include <phantomx_arm_hw/arbotix.hpp>

#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(rclcpp::get_logger("PhantomXArmHardware"), x)
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("PhantomXArmHardware"), x)

bool Arbotix::open(std::string &port_name)
{
  return port_.open(port_name);
}

bool Arbotix::is_port_open()
{
  return port_.is_open();
}

bool Arbotix::close()
{
  return port_.close();
}

bool Arbotix::read(unsigned char *ids, unsigned char n_ids, unsigned char addr, unsigned char n_bytes, unsigned char *buf)
{
  int rs = n_ids*n_bytes;
  unsigned char packet[n_ids+8] = {255, 255, 254, (unsigned char)(n_ids+4), 132, addr, n_bytes};
  unsigned char response[rs+6];
  int n;

  // Copy ids into packet.
  memcpy(&packet[7], ids, n_ids);

  // Set checkum.
  packet[n_ids+7] = checksum(&packet[2], n_ids+5);

  // Write packet to port.
  n = port_.write(packet, n_ids+8);
  if (n != n_ids+8)
  {
    ROS_ERROR_STREAM("Could not write to port");
    return false;
  }
  
  // Read response. Note that we do not handle status packet errors. These will cause a timeout.
  n = port_.read(response, rs+6, 0, 100000);
  if (n != 6+n_ids*n_bytes)
  {
    if (n == 6)
    {
      // Status packet
      if (checksum(&response[2], 3) != response[5])
      {
        ROS_ERROR_STREAM("Checksum error on status: expected " << (int)checksum(&response[2], 3) << ", got " << (int)response[5]);
        return false;
      }
      
      if (response[4] == 16)
        ROS_INFO_STREAM("Arbotix board reported checksum error");
      else
        ROS_INFO_STREAM("Arbotix board reported error: " << (int)response[4]);

      return false;
    }
  
    ROS_INFO_STREAM("Could not read from port: received " << n << " bytes, expected " << rs+6);
    return false;
  }
  
  // Verify checksum.
  if (checksum(&response[2], rs+3) != response[rs+5])
  {
    ROS_ERROR_STREAM("Checksum error: expected " << (int)checksum(&response[2], rs+3) << ", got " << (int)response[rs+5]);
    port_.flush();
    return false;
  }

  // Copy response into buffer.
  memcpy(buf, &response[5], rs);

  return true;
}

bool Arbotix::write(unsigned char *ids, unsigned char n_ids, unsigned char addr, unsigned char n_bytes, unsigned char *buf)
{
  int ps = n_ids*(n_bytes+1);
  unsigned char packet[ps+8] = {255, 255, 254, (unsigned char)(ps+4), 131, addr, n_bytes};
  int n;

  // Copy id and data into packet for all ids.
  for (size_t ii=0; ii != n_ids; ++ii)
  {
            packet[ii*(n_bytes+1)+7] = ids[ii];
    memcpy(&packet[ii*(n_bytes+1)+8], &buf[ii*n_bytes], n_bytes);
  }

  // Set checkum.
  packet[ps+7] = checksum(&packet[2], ps+5);
  
  // Write packet to port.
  n = port_.write(packet, ps+8);
  if (n != ps+8)
  {
    ROS_ERROR_STREAM("Could not write to port");
    return false;
  }
  
  return true;
}
