#include <cstdio>
#include <cstdlib>
#include "Display.h"
#include "Lidar.h"

int main(void)
{
	robotik::Lidar lidar("/dev/ttyUSB0", 115200);
	lidar.startMotorAndScan();
    robotik::Display disp(1000, 500,  10*1000, 5*1000,   5*1000, 2.5*1000, &lidar); 
    return disp.start_qt_display();
}
