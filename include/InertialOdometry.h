#include "vex.h"

class InertialOdom
{
    private:

        //Divide this number by the accelerometer number to get in/s^2
        const double G_TO_UNITS = 386.0885826772;

        //The value of pi
        const double PI = atan(1) * 4;

        //Turns degrees into Radians
        double deg2Rad(double deg)
        {
            return deg * (PI / 180.0);
        }

        //Turns Radians into degrees
        double rad2Deg(double rad)
        {
            return rad * (180.0 / PI);
        }

        //Unit Vector
        struct Vector
        {
            double x = 0;
            double y = 0;
            double z = 0;
        };

        inertial & inertialSensor;

    public:

        //inertial sensor;
        double dt;

        double matrix[3][3];

        Vector ut_Acceleration; //Acceleration untransformed
        Vector tr_OffsetAcceleration; //Gravity relative to the fields perspective
        Vector tr_Acceleration; //Acceleration transformed to the fields perspective

        Vector tr_Velocity; //Velocity relative to the fields perspective
        Vector tr_Displacement; //Displacement relative to the fields perspective

        Vector angularVelocity; //Rate from the gyro sensor
        Vector s_angle; //Sensors direction

        InertialOdom(inertial & sensor, double dt) : inertialSensor(sensor)
        {
          this->dt = dt;

          inertialSensor.calibrate();
          while(inertialSensor.isCalibrating()){}

          setInertialData();

          updateMatrix();

          transformVector();

          tr_OffsetAcceleration = tr_Acceleration;
        }

        void setInertialData();

        void updateMatrix();

        void transformVector();

        Vector integrateVector(Vector a, Vector A, double dt);

        void findDisplacement();
};
