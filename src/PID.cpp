#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

#ifdef TWIDDLE_ON
void PID::Init(double Kp, double Ki, double Kd, std::ofstream *ofs_ptr) {
#else
void PID::Init(double Kp, double Ki, double Kd) {
#endif
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
  
    p_error = 0;
    i_error = 0;
    p_error = 0;

    // Twiddel parameters
    tol = 0.01;
    twiddle_step_size[0] = 0.01;
    twiddle_step_size[1] = 0.0001;
    twiddle_step_size[2] = 0.01;
    twiddle_idx = 0;
    is_pos = true;
    update_counter = 0;
    best_cte2_error = 1e+07; // initialize the best_cte2_error as a very bit error
    cur_acc_cte2_error = 0;
  
#ifdef TWIDDLE_ON
    ofsPtr = ofs_ptr;
#endif
}

void PID::Reset(){
  p_error = i_error = d_error = 0;
  update_counter = 0;
  cur_acc_cte2_error = 0;
}

void PID::UpdateError(double cte) {
    d_error = cte-p_error;
    p_error = cte;
    i_error += cte;
    // for Twiddle
    update_counter++;
    cur_acc_cte2_error += cte*cte;
    //cout << "cur_acc_cte2_error = " << cur_acc_cte2_error << endl;
}

double PID::getSteer() {
    float steer_value = -Kp*p_error - Ki*i_error - Kd*d_error;
    steer_value = steer_value<-1?-1:steer_value;
    steer_value = steer_value>1?1:steer_value;
    return steer_value;
}

double PID::TotalError() {
  std::cout << "\tNormalize acc cte2 with " << update_counter << std::endl;
  return cur_acc_cte2_error/update_counter;
}

void PID::Twiddle() {
  std::cout << "\tKp = " << Kp << "; Ki = " << Ki << "; Kd = " << Kd << std::endl;
  std::cout << "\tdKp = " << twiddle_step_size[0] << "; dKi = " << twiddle_step_size[1] << "; dKd = " << twiddle_step_size[2] << std::endl;
  std::cout << "\tadjust index = " << twiddle_idx << std::endl;
  
  double cur_tol = twiddle_step_size[0]+twiddle_step_size[1]*100+twiddle_step_size[2];
  if (cur_tol<tol)
  {
    cout << "Convergence of Twiddle!\nKp=" << Kp << "\tKi=" << Ki << "\tKd=" << Kd << endl;
    return;
  }
  
#ifdef TWIDDLE_ON
  *ofsPtr << Kp << " " << Ki << " " << Kd << std::endl;
  *ofsPtr << twiddle_step_size[0] << " " << twiddle_step_size[1] << " " << twiddle_step_size[2] << std::endl;
#endif
  
  double *K_target;
  if (twiddle_idx==0) { // Kp
    K_target = &Kp;
  } else if (twiddle_idx==1) {  // Ki
    K_target = &Ki;
  } else {  // Kd
    K_target = &Kd;
  }
  
  double cur_cte2_error = TotalError();
  std::cout << "\tcur_cte2_error = " << cur_cte2_error << "; best_cte2_error = " << best_cte2_error << std::endl;
  
  #ifdef TWIDDLE_ON
  *ofsPtr << cur_tol << " " << cur_cte2_error << " " << best_cte2_error << std::endl;
  #endif
  
  std::cout << "\tDirection is_pos = " << is_pos << std::endl;
  if (is_pos) { // positive direction
    if (cur_cte2_error < best_cte2_error) { // cur_error is smaller
      best_cte2_error = cur_cte2_error;
      twiddle_step_size[twiddle_idx] *= 1.1;
      //*K_target += twiddle_step_size[twiddle_idx];
    }
    else {  // set to search negative direction
      *K_target -= 2*twiddle_step_size[twiddle_idx];
      is_pos = false;
    }
  }
  else {  // negative direction
    if (cur_cte2_error < best_cte2_error) { // cur_error is smaller
      best_cte2_error = cur_cte2_error;
      twiddle_step_size[twiddle_idx] *= 1.1;
    }
    else {
      *K_target += twiddle_step_size[twiddle_idx];  // this negative direction is not better than original settings, so make it back to original settings
      twiddle_step_size[twiddle_idx] *= 0.9;
    }
    is_pos = true;
  }
  
  if (is_pos) {
    twiddle_idx = (twiddle_idx+1)%3;
    if (twiddle_idx==0) { // Kp
      K_target = &Kp;
    } else if (twiddle_idx==1) {  // Ki
      K_target = &Ki;
    } else {  // Kd
      K_target = &Kd;
    }
    *K_target += twiddle_step_size[twiddle_idx];
  }
}




