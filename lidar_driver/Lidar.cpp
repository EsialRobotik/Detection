#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <time.h> 
#include <math.h>
#include <unistd.h>
#include "Lidar.h"
#include "include/rplidar.h"

#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

using namespace rp::standalone::rplidar;

namespace robotik
{
  Lidar::Lidar(std::string port_name, uint32_t serial_baud_rate) : running_(false)
  {
    rplidar_response_device_health_t healthinfo;
    rplidar_response_device_info_t devinfo;

    lidar_driver_ = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    if(!lidar_driver_)
    {
      fprintf(stderr, "insufficent memory\n");
      exit(1);
    }

    if (IS_FAIL(lidar_driver_->connect(port_name.c_str(), serial_baud_rate))) 
    {
      fprintf(stderr, "Unable to open port %s @ %d bps\n",port_name.c_str(), serial_baud_rate );
      exit(1);
    }
    
    uint32_t res = lidar_driver_->getDeviceInfo(devinfo);

    if (IS_FAIL(res)) 
    {
      if (res == RESULT_OPERATION_TIMEOUT)
      {
        fprintf(stderr, "Lidar communication timed out\n");
        exit(1);
      }
      else
      {
        fprintf(stderr, "Lidar communication unexpected error\n");
        exit(1);
      }
    }

    res = lidar_driver_->getHealth(healthinfo);
    if (IS_OK(res) && (healthinfo.status == RPLIDAR_STATUS_ERROR || healthinfo.status == RPLIDAR_STATUS_WARNING)   )
    {
      fprintf(stderr, "Lidar health status is %s\n", healthinfo.status == RPLIDAR_STATUS_ERROR  ? "error" : "warning");
      exit(1);
    }
    else if ( IS_FAIL(res))
    {
      fprintf(stderr, "cannot retrieve lidar health status.Code: %x\n", res);
      exit(1);
    }
  }

  Lidar::~Lidar()
  {
    lidar_driver_->stop();
    lidar_driver_->stopMotor();

    RPlidarDriver::DisposeDriver(lidar_driver_);

  }

  void Lidar::startMotorAndScan()
  {
    lidar_driver_->startMotor();

    lidar_driver_->setMotorPWM(MAX_MOTOR_PWM);

     if (IS_FAIL(lidar_driver_->startScan()))
    {
      fprintf(stderr, "cannot start lidar scan\n");
      exit(1);
    }
    running_ = true;

  }

  int Lidar::get360degreesXYsamples( std::vector<robotik::lidar_xy_sample_t> &samples)
  {
    rplidar_response_measurement_node_t nodes[450];
    size_t   count = _countof(nodes);
     
    if (IS_OK(lidar_driver_->grabScanData(nodes, count)) )
    {
      samples.clear();

      for(unsigned int i=0; i<count; i++)
      {
        if (!nodes[i].distance_q2) 
          continue;

        robotik::lidar_xy_sample_t sampleXY;
        float angle_rad =  float(nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f * M_PI/180.0;
        float dist_mm = nodes[i].distance_q2/4.0f;
        sampleXY.x = cos(angle_rad) * dist_mm;
        sampleXY.y = sin(angle_rad) * dist_mm;    
        samples.push_back(sampleXY);
      }
    }
    return count;
  }
 

}
