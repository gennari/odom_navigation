#include <ros/ros.h>
#include <time.h>
#include <ros/time.h>

using namespace std;

namespace pid
{

  class PID
  {

    public:

      PID()
      {
        int_err = 0;
      }

      ~PID(){};


      void paramInit(double k_p, double k_d, double k_i)
      {
        k_prop = k_p;
        k_der = k_d;
        k_int = k_i;
      }

      inline double getCommand(double error, double error_dot, double dT)
      {

        int_err = int_err + dT*error;

        windupControl();

        return k_prop*error + k_der*error_dot + k_int*int_err;
      } 

    private:

      double k_prop, k_der, k_int;
      double int_err;
      double windup_upper, windup_slower;

      inline void windupControl()
      {
        if(int_err > windup_upper) int_err = windup_upper;
        else if(int_err < windup_slower) int_err = windup_slower;
      }


  };

}

