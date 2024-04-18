#include "InertialOdometry.h"


void InertialOdom::getInertialData(inertial sensor, Vector * acc, Vector * angVel, Vector * ang)
{
  acc->x = sensor.acceleration(xaxis) / G_TO_UNITS;
  acc->y = sensor.acceleration(yaxis) / G_TO_UNITS;
  acc->z = sensor.acceleration(zaxis) / G_TO_UNITS;

  angVel->x = sensor.gyroRate(xaxis, dps);
  angVel->y = sensor.gyroRate(yaxis, dps);
  angVel->z = sensor.gyroRate(zaxis, dps);

  ang->x = sensor.roll();
  ang->y = sensor.pitch();
  ang->z = sensor.yaw();
}

void InertialOdom::transformCoordinate(inertial sensor, Vector * ptrVec)
{
  
}