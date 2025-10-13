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
  unsigned char packet[n_ids+8] = {255, 255, 254, (unsigned char)(n_ids+4), 132, addr, n_bytes};
  unsigned char response[n_ids*n_bytes+6];
  int n;

  memcpy(&packet[7], ids, n_ids);
  packet[n_ids+7] = checksum(&packet[2], n_ids+7);

  n = port_.write(packet, n_ids+8);
  if (n != n_ids+8)
  {
    ROS_ERROR_STREAM("Could not write to port");
    return false;
  }
    
  n = port_.read(response, 6+n_ids*n_bytes, 0, 100000);
  if (n != 6+n_ids*n_bytes)
  {
    ROS_INFO_STREAM("Could not read from port: received " << n << " bytes, expected " << 6+n_ids*n_bytes);
    return false;
  }

  if (checksum(&response[2], n_ids*n_bytes+3) != response[n_ids*n_bytes+5])
  {
    ROS_ERROR_STREAM("Checksum error");
    return false;
  }

  memcpy(buf, &response[5], n_ids*n_bytes);

  return true;
}

bool Arbotix::write(unsigned char *ids, unsigned char n_ids, unsigned char addr, unsigned char n_bytes, unsigned char *buf)
{
  int ps = n_ids*(n_bytes+1);
  unsigned char packet[ps+8] = {255, 255, 254, (unsigned char)(ps+4), 131, addr, n_bytes};
  int n;

  for (size_t ii=0; ii != n_ids; ++ii)
  {
            packet[ii*(n_bytes+1)+7] = ids[ii];
    memcpy(&packet[ii*(n_bytes+1)+8], &buf[ii*n_bytes], n_bytes);
  }
  packet[ps+7] = checksum(&packet[2], ps+5);
  
  n = port_.write(packet, ps+8);
  if (n != ps+8)
  {
    ROS_ERROR_STREAM("Could not write to port");
    return false;
  }
  
  return true;
}

