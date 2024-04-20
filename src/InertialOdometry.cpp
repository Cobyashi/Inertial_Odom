#include "vex.h"

void InertialOdom::setInertialData()
{
  ut_Acceleration.x = inertialSens.acceleration(xaxis) / G_TO_UNITS;
  ut_Acceleration.y = inertialSens.acceleration(yaxis) / G_TO_UNITS;
  ut_Acceleration.z = inertialSens.acceleration(zaxis) / G_TO_UNITS;

  angularVelocity.x = inertialSens.gyroRate(xaxis, dps);
  angularVelocity.y = inertialSens.gyroRate(yaxis, dps);
  angularVelocity.z = inertialSens.gyroRate(zaxis, dps);

  s_angle.x = inertialSens.roll();
  s_angle.y = inertialSens.pitch();
  s_angle.z = inertialSens.yaw();
}

//updates transformation matrix. precursor to vector relative to the field
void InertialOdom::updateMatrix()
{
  double a = -(deg2Rad(s_angle.z));
  double b = -(deg2Rad(s_angle.y));
  double c = -(deg2Rad(s_angle.x));

  matrix[0][0] = cos(a)*cos(b);
  matrix[0][1] = (cos(a)*sin(b)*sin(c))-(sin(a)*cos(c));
  matrix[0][2] = (cos(a)*sin(b)*cos(c))+(sin(a)*sin(c));

  matrix[1][0] = sin(a)*cos(b);
  matrix[1][1] = (sin(a)*sin(b)*sin(c))+(cos(a)*cos(c));
  matrix[1][2] = (sin(a)*sin(b)*cos(c))-(cos(a)*sin(c));

  matrix[2][0] = -sin(b);
  matrix[2][1] = cos(b)*sin(c);
  matrix[2][2] = cos(b)*cos(c);

}

//apply the transformation, making the vector relative to the field
void InertialOdom::transformVector()
{
  tr_Acceleration.x = (ut_Acceleration.x * matrix[0][0]) + (ut_Acceleration.y * matrix[0][1]) + (ut_Acceleration.z * matrix[0][2]);
  tr_Acceleration.y = (ut_Acceleration.x * matrix[1][0]) + (ut_Acceleration.y * matrix[1][1]) + (ut_Acceleration.z * matrix[1][2]);
  tr_Acceleration.z = (ut_Acceleration.x * matrix[2][0]) + (ut_Acceleration.y * matrix[2][1]) + (ut_Acceleration.z * matrix[2][2]);
}

InertialOdom::Vector InertialOdom::integrateVector(Vector a, Vector A, double dt)
{
  a.x = a.x + (A.x * dt);
  a.y = a.y + (A.y * dt);
  a.z = a.z + (A.z * dt);

  return a;
}

void InertialOdom::initialize(double dt)
{
  this->dt = dt;

  inertialSens.calibrate();
  waitUntil(!inertialSens.isCalibrating());

  tr_Displacement.x = 0;
  tr_Displacement.y = 0;
  tr_Displacement.z = 0;

  tr_Velocity.x = 0;
  tr_Velocity.y = 0;
  tr_Velocity.z = 0;

  setInertialData();

  updateMatrix();

  transformVector();

  tr_OffsetAcceleration = tr_Acceleration;
}

void InertialOdom::findDisplacement()
{
  //transform the acceleration vector
  setInertialData();
  updateMatrix();
  transformVector();

  //apply the offset acceleration
  tr_Acceleration.x = tr_Acceleration.x - tr_OffsetAcceleration.x;
  tr_Acceleration.y = tr_Acceleration.y - tr_OffsetAcceleration.y;
  tr_Acceleration.z = tr_Acceleration.z - tr_OffsetAcceleration.z;

  //integrate to velocity
  tr_Velocity = integrateVector(tr_Velocity, tr_Acceleration, dt);

  //integrate to displacement
  tr_Displacement = integrateVector(tr_Displacement, tr_Velocity, dt);
}