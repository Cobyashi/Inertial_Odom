#include "vex.h"

using namespace vex;

const double G_TO_UNITS = 386.0885826772; //divide this number by the accelerometer number to get in/s^2

const double PI = atan(1) * 4;

double deg2Rad(double rad)
{
  return (rad * PI) / 180.0;
}

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
            double x;
            double y;
            double z;
        };

    public:

        void getInertialData(inertial sensor, Vector * acc, Vector * angVel, Vector * ang);

        void transformCoordinate(inertial sensor, Vector * ptrVec);
};
