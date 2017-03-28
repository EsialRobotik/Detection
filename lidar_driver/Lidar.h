/*
 * DisplayPoints.h
 *
 *  Created on: 22 févr. 2017
 *      Author: T0132956
 */

#ifndef NIARF_LIDAR_H_
#define NIARF_LIDAR_H_

#include <cstring>
#include <stdint.h>
#include <vector>

namespace rp { namespace standalone { namespace rplidar { class RPlidarDriver; } } };

namespace robotik
{
  struct lidar_xy_sample_t
  {
    int x;
    int y;
    lidar_xy_sample_t(int x_val, int y_val) : x(x_val), y(y_val) {}
    lidar_xy_sample_t()  {}
  } ;


  class Lidar
  {

    public:
      Lidar(std::string port_name, uint32_t serial_baud_rate);

      virtual ~Lidar();

      void startMotorAndScan();

      int get360degreesXYsamples( std::vector<robotik::lidar_xy_sample_t> &samples);


      bool is_running(){ return running_;};
    private:
      rp::standalone::rplidar::RPlidarDriver *lidar_driver_;
      bool running_;
  };
}
#endif /* NIARF_LIDAR_H_ */
