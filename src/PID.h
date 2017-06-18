#ifndef PID_H
#define PID_H

#include <iostream>
#include <fstream>
#include <vector>

//#define TWIDDLE_ON

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * Twiddel coeffifients
   */
  double tol; // tolerance
  double twiddle_step_size[3];  // p,i,d step_size
  int twiddle_idx;
  bool is_pos;  // positive and negative twiddle directions

  int update_counter;
  double best_cte2_error; // best "Average" (sum of cte^2 error) so far
  double cur_acc_cte2_error;  // cur sum of cte^2 error

  #ifdef TWIDDLE_ON
  std::ofstream *ofsPtr;
  #endif
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
#ifdef TWIDDLE_ON
  void Init(double Kp, double Ki, double Kd, std::ofstream *ofs_ptr);
#else
  void Init(double Kp, double Ki, double Kd);
#endif

  void Reset();
  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
  
  /*
   * Calculate steer angle
   */
  double getSteer();

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * Twiddle
  */
  void Twiddle();
};

#endif /* PID_H */
