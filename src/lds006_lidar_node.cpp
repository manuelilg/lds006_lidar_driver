#include <ros/ros.h>
#include <signal.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <array>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

int32_t comHandle = 0;

//void sigintHandler(int sig) {
//  ros::shutdown();
//}

bool readFromIO(std::array<uint8_t, 21>& rxBuffer) {
  ssize_t lenToRead = rxBuffer.size();
  do {
    const ssize_t len = read(comHandle, rxBuffer.data() + (rxBuffer.size() - lenToRead), lenToRead);
    // check if synced up to start byte 0xFA and if not sync up
    if (lenToRead == rxBuffer.size() && len > 0) {
      if (rxBuffer.at(0) != 0xFA) {
        ssize_t rangeBegin = len;
        for (ssize_t i = 1; i < len; ++i) {
          if (rxBuffer.at(i) == 0xFA) {
            rangeBegin = i;
          }
        }
        std::copy(rxBuffer.begin() + rangeBegin, rxBuffer.begin() + len, rxBuffer.begin());
        lenToRead = rxBuffer.size() - (len - rangeBegin);
      }
    }
    else if (len == -1) {
      return false;
    }
    else {
      lenToRead -= len;
    }
    //std::cout << "read " << lenToRead << std::endl;
  } while (lenToRead > 0);
  return true;
}

void readAndPrint() {
  std::array<uint8_t, 21> rxBuffer{};
  readFromIO(rxBuffer);

  for (uint8_t val : rxBuffer) {
    std::cout << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(val) << ' ';
  }
  std::cout << std::endl;
}

void readAndParsePacket(sensor_msgs::LaserScan& laserScanMsg) {
  std::array<uint8_t, 21> packet{};
  const bool success = readFromIO(packet);
  if (success) {
    const uint8_t angleRaw = packet.at(1);
    if (0xA0 <= angleRaw && angleRaw <= 0xF9) {
      const uint16_t angle_base = (angleRaw - 0xA0) * 4;
      //std::cout << angle << std::endl;
      for (size_t i = 0; i < 4; ++i) {
        const size_t offset = 4 + i * 4;
        const size_t angle = angle_base + i;
        const uint16_t distance = packet.at(offset) | (packet.at(offset + 1) << 8);
        //std::cout << int(packet.at(offset)) << ' ' << int(packet.at(offset + 1)) << std::endl;
        const uint16_t reflection = packet.at(offset + 2) | (packet.at(offset + 3) << 8);

        if (reflection > 0 && distance < 15000) {
          laserScanMsg.ranges.at(angle) = distance / 1000.0;
          laserScanMsg.intensities.at(angle) = reflection;
          //std::cout << angle+2*i << '\t' << distance << std::endl;
        }
        else {
          //laserScanMsg.ranges.push_back(std::numeric_limits<float>::quiet_NaN());
          //laserScanMsg.intensities.push_back(std::numeric_limits<float>::quiet_NaN());
        }

        //std::cout << angle+2*i << std::endl;
        //if (reflection > 0 && distance < 200) {
        //  std::cout << distance << std::endl;
        //}
      }
    }
    //for (uint8_t val : packet) {
    //  std::cout << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(val) << ' ';
    //}
    //std::cout << std::endl;
  }
  else {
    std::cout << "readFromIO failed!" << std::endl;
  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "lds006_lidar_node");
  ROS_INFO("Hello from lds006 driver");
  ros::NodeHandle nh;

  int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);
  comHandle = open("/dev/serial0", flags);
  if (comHandle == -1) {
    ROS_ERROR("Can not open serial port");
    return -1;
  }

  struct termios options;
  if (-1 == tcgetattr(comHandle, &options))
  {
    close(comHandle);
    ROS_ERROR("CmdInterfaceLinux::Open tcgetattr error!");
    return -1;
  }

  options.c_cflag |= (tcflag_t)(CLOCAL | CREAD | CS8 | CRTSCTS);
  options.c_cflag &= (tcflag_t) ~(CSTOPB | PARENB | PARODD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                    ISIG | IEXTEN);    //|ECHOPRT
  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | INLCR | IGNCR | ICRNL | IGNBRK);

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  cfsetispeed(&options, B115200);

  if (tcsetattr(comHandle, TCSANOW, &options) < 0)
  {
      std::cout << "CmdInterfaceLinux::Open tcsetattr error!" << std::endl;
      close(comHandle);
      return false;
  }

  tcflush(comHandle, TCIFLUSH);

  std::string startLidarCmd{"startlds$"};
  write(comHandle, startLidarCmd.c_str(), startLidarCmd.length());

  ROS_INFO("startlds$ send");

  //signal(SIGINT, sigintHandler);

  //std::cout << std::hex;

  ros::Publisher lidarPub = nh.advertise<sensor_msgs::LaserScan>("LiDAR/LDS006", 1);

  while(ros::ok()) {
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = "lidar_frame";
    msg.angle_min = 0;
    msg.angle_max = 2.0 * M_PI;
    msg.range_min = 100 / 1000.0;
    msg.range_max = 15000 / 1000.0;
    msg.angle_increment = 2.0 * M_PI * 1.0/360.0;
    //msg.time_increment = ??
    //msg.scan_time = ??

    msg.ranges.resize(360, std::numeric_limits<float>::quiet_NaN());
    msg.intensities.resize(360, std::numeric_limits<float>::quiet_NaN());
    for(int i = 0; i < 90; ++i) {
      readAndParsePacket(msg);
      //readAndPrint();
    }
    //std::cout << "############" << std::endl;

    msg.header.stamp = ros::Time::now();
    lidarPub.publish(msg);
  }


  if (comHandle != 0) {
    std::string stopLidarCmd{"stoplds$"};
    write(comHandle, stopLidarCmd.c_str(), stopLidarCmd.length());
    //fsync(comHandle);
    close(comHandle);
  }

  return 0;
}
