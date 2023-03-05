#include <ros/ros.h>
#include <signal.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <array>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

int32_t comHandle = 0;

int nrOfPackages = 0;
uint64_t speedSum = 0U;

//void sigintHandler(int sig) {
//  ros::shutdown();
//}

void printArray(std::array<uint8_t, 22> rxBuffer, size_t length) {
//  std::cout << std::setw(2) << std::setfill('0') << std::right << std::hex << std::uppercase;
  for (size_t i = 0U; i < length; ++i) {
    std::cout << std::setw(2) << std::setfill('0') << std::right << std::hex << std::uppercase << static_cast<int>(rxBuffer[i]) << ' ';
  }
  std::cout << std::dec << std::endl;
}

bool readFromIO(std::array<uint8_t, 22>& rxBuffer) {
  //std::cout << "=======" << std::endl;
  ssize_t lenToRead = rxBuffer.size();
  do {
    const ssize_t len = read(comHandle, rxBuffer.data() + (rxBuffer.size() - lenToRead), lenToRead);
    //std::cout << "len: "  << len << std::endl;
    // check if synced up to start byte 0xFA and if not sync up
    if (len > 0) {
      if (rxBuffer.at(0) != 0xFA) {
        //std::cout << "1" << std::endl;
        ssize_t rangeBegin = len;
        for (ssize_t i = 1; i < len; ++i) {
          if (rxBuffer.at(i) == 0xFA) {
            rangeBegin = i;
          }
        }
        std::copy(rxBuffer.begin() + rangeBegin, rxBuffer.begin() + len, rxBuffer.begin());
        lenToRead = rxBuffer.size() - (len - rangeBegin);
      }
      else {
        //std::cout << "2" << std::endl;
        lenToRead -= len;
      }
    }
    else if (len == -1) {
      //std::cout << "3" << std::endl;
      return false;
    }

    /*if (len == 22U) {
      std::cout << "\n";
    }
    else {
      std::cout << "x\n";
    }*/

    //std::cout << "lenToRead: " << lenToRead << std::endl;
    //printArray(rxBuffer, rxBuffer.size() - lenToRead);
  } while (lenToRead > 0);
  return true;
}

void readAndPrint() {
  std::array<uint8_t, 22> rxBuffer{};
  const ssize_t len = read(comHandle, rxBuffer.data(), rxBuffer.size());
  printArray(rxBuffer, len);
}

void readPackageAndPrint() {
  std::array<uint8_t, 22> rxBuffer{};
  readFromIO(rxBuffer);
  printArray(rxBuffer, rxBuffer.size());
}

void fillHeaderConstantPart(sensor_msgs::LaserScan& msg) {
  msg.header.frame_id = "lidar_frame";
  msg.range_min = 100 / 1000.0;
  msg.range_max = 15000 / 1000.0;
  msg.angle_increment = 2.0 * M_PI * 1.0/360.0;
  msg.time_increment = 1.0/5.0/360.0; // time between samples, rotation speed 5Hz
}

void fillHeaderVariablePart(sensor_msgs::LaserScan& msg, const int angle, const int nrOfSamples) {
  msg.angle_min = angle * msg.angle_increment;
  msg.angle_max = (angle + nrOfSamples) * msg.angle_increment;
  msg.scan_time = nrOfSamples * msg.time_increment; // duration for scan in seconds (for 360deg time for one rotation)
}

void readAndParsePacket(sensor_msgs::LaserScan& laserScanMsg) {
  std::array<uint8_t, 22> packet{};
  const bool success = readFromIO(packet);
  if (success) {
    // package structure: https://www.jentsch.io/lds-006-lidar-sensor-reverse-engineering/
    const uint8_t angleRaw = packet.at(1);
    if (0xA0 <= angleRaw && angleRaw <= 0xF9) {
      const uint16_t angle_base = (angleRaw - 0xA0) * 4;
      fillHeaderConstantPart(laserScanMsg);
      fillHeaderVariablePart(laserScanMsg, angle_base, 4);
      //std::cout << angle << std::endl;
      const uint16_t rev = packet.at(2) | (packet.at(3) << 8);
      speedSum += rev;
      //std::cout << rev << "\n";
      for (size_t i = 0; i < 4; ++i) {
        const size_t offset = 4 + i * 4;
        const size_t angle = angle_base + i;
        const uint16_t distance = packet.at(offset) | (packet.at(offset + 1) << 8);
        //std::cout << int(packet.at(offset)) << ' ' << int(packet.at(offset + 1)) << std::endl;
        const uint16_t reflection = packet.at(offset + 2) | (packet.at(offset + 3) << 8);

        if (reflection > 0 && distance < 15000) {
          //laserScanMsg.ranges.at(angle) = distance / 1000.0;
          //laserScanMsg.intensities.at(angle) = reflection;
          laserScanMsg.ranges.push_back(distance / 1000.0);
          laserScanMsg.intensities.push_back(reflection);
          //std::cout << angle+2*i << '\t' << distance << std::endl;
        }
        else {
          laserScanMsg.ranges.push_back(std::numeric_limits<float>::quiet_NaN());
          laserScanMsg.intensities.push_back(std::numeric_limits<float>::quiet_NaN());
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

void printNrOfBytesInUartBuffer(const ros::TimerEvent& event) {
  const double rate = nrOfPackages / (event.current_real - event.last_real).toSec();

  int nrOfBytes = 0;
  ioctl(comHandle, FIONREAD, &nrOfBytes);
  std::cout << "package rate: " << rate << ", rev: " << rate/90.0 << ", rev2: " << speedSum/60.0/nrOfPackages/100.0 << ", nr of bytes in buffer: " << nrOfBytes << "\n";
  nrOfPackages = 0;
  speedSum = 0U;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "lds006_lidar_node");
  ROS_INFO("Hello from lds006 driver");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

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

  ros::Rate rate(455); // 5 rotations per second and 90 packages per rotation + 1%

  //ros::Timer uartBufferTimer = nh.createTimer(ros::Duration(1.0), printNrOfBytesInUartBuffer);

  float cutAngle;
  nh_priv.param("cut_angle", cutAngle, -0.1f);

  while(ros::ok()) {
    sensor_msgs::LaserScan laserScan;
    /*
    msg.header.frame_id = "lidar_frame";
    msg.angle_min = 0;
    msg.angle_max = 2.0 * M_PI;
    msg.range_min = 100 / 1000.0;
    msg.range_max = 15000 / 1000.0;
    msg.angle_increment = 2.0 * M_PI * 1.0/360.0;
    msg.time_increment = 1.0/5.0/360.0; // time between samples, rotation speed 5Hz
    msg.scan_time = 1.0/5.0; // duration for scan in seconds (for 360deg time for one rotation)

    msg.ranges.resize(360, std::numeric_limits<float>::quiet_NaN());
    msg.intensities.resize(360, std::numeric_limits<float>::quiet_NaN());
    */

    const int nrOfSegments = 90;
    for(int i = 0; i < nrOfSegments;) {
      sensor_msgs::LaserScan laserScanSegment;
      laserScanSegment.ranges.reserve(4);
      laserScanSegment.intensities.reserve(4);

      //ros::spinOnce();
      rate.sleep();
      readAndParsePacket(laserScanSegment);

      if ((laserScanSegment.angle_min == cutAngle) || ((cutAngle < 0.0f) && (laserScan.ranges.empty()))) {
        laserScan = std::move(laserScanSegment);
        laserScan.ranges.reserve(4 * nrOfSegments);
        laserScan.intensities.reserve(4 * nrOfSegments);
        if (laserScanSegment.angle_min > 0.0f) {
          laserScan.angle_min -= 2 * M_PI;
          laserScan.angle_max -= 2 * M_PI;
        }

        ++i;
      }
      else if (!laserScan.ranges.empty()) {
        std::move(std::begin(laserScanSegment.ranges), std::end(laserScanSegment.ranges), std::back_inserter(laserScan.ranges));
        std::move(std::begin(laserScanSegment.intensities), std::end(laserScanSegment.intensities), std::back_inserter(laserScan.intensities));
        laserScan.scan_time += laserScanSegment.scan_time;
        laserScan.angle_max = laserScanSegment.angle_max;
        if ((laserScan.angle_max > cutAngle) && (laserScan.angle_min > 0.0f)) {
          laserScan.angle_max -= 2 * M_PI;
        }

        ++i;
      }
      else {
         i = 0;
      }
      //readAndPrint();
      //++nrOfPackages;
      //ros::spinOnce();
    }
    //std::cout << "############" << std::endl;
    //laserScan.angle_max = M_PI;
    laserScan.header.stamp = ros::Time::now();
    lidarPub.publish(laserScan);

    //ros::spinOnce();
  }

  //uartBufferTimer.stop();

  if (comHandle != 0) {
    std::string stopLidarCmd{"stoplds$"};
    write(comHandle, stopLidarCmd.c_str(), stopLidarCmd.length());
    //fsync(comHandle);
    close(comHandle);
  }

  return 0;
}
