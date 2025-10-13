#include <phantomx_arm_hw/arbotix.hpp>

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

  memcpy(&packet[7], buf, n_ids);
  packet[n_ids+7] = checksum(packet, n_ids+7);

  if (port_.write(packet, n_ids+8) != n_ids+8)
    return false;

  if (port_.read(response, 6+n_ids*n_bytes, 0, 100000) != 6+n_ids*n_bytes)
    return false;

  if (checksum(&response[2], n_ids*n_bytes+3) != response[n_ids*n_bytes+5])
  {
    printf("Checksum error");
    return false;
  }

  memcpy(buf, &response[5], n_ids*n_bytes);

  return true;
}

bool Arbotix::write(unsigned char *ids, unsigned char n_ids, unsigned char addr, unsigned char n_bytes, unsigned char *buf)
{
  int ps = n_ids*(n_bytes+1);
  unsigned char packet[ps+8] = {255, 255, 254, (unsigned char)(ps+4), 131, addr, n_bytes};

  for (size_t ii=0; ii != n_ids; ++ii)
  {
            packet[ii*(n_bytes+1)+7] = ids[ii];
    memcpy(&packet[ii*(n_bytes+1)+8], &buf[ii*n_bytes], n_bytes);
  }
  packet[ps+7] = checksum(&packet[2], ps+5);
  
  return port_.write(packet, ps+8);
}

