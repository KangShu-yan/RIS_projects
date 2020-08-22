#ifndef __COMP417_PID_H__
#define __COMP417_PID_H__


class PID {
 public:
  PID(double Kp_, double Td_, double Ti_, double dt_) {
    // todo: write this
    Kp = Kp_;
    Td = Td_;
    Ti = Ti_;
    dt = dt_;
  };
  
  void update_control(double current_error) {
  	double p_gain=0,i_gain=0,d_gain=0;
    // todo: write this
     current_deriv_error = (curr_error - previous_error) / dt;

     //steering angle = P gain + D gain + I gain
     p_gain = Kp * curr_error;

     i_gain = sum_error  + Ti * curr_error * dt;
     
     if(i_gain>1){
     	i_gain = 1;
     }
     if(i_gain<-1){
     	i_gain = -1;
     }
     sum_error = i_gain;
     d_gain = Td * current_deriv_error;

     //PID control
     control = p_gain + d_gain + i_gain; //= control
//   	 if(control>1.5){
//   	 	control = 1.5;
//   	 }
//   	 if(control<-1.5){
//   	 	control = -1.5;
//   	 }

     // update error
     previous_error = curr_error;
     curr_error = current_error;
     previous_deriv_error = current_deriv_error;
  };
	
  double get_control() {
    return control;
  };

 private:
  double Kp;
  double Td;
  double Ti;

  double curr_error;
  double previous_error;

  double sum_error;
  
  double current_deriv_error;
  double previous_deriv_error;
  double control;
  double dt;
  uint32_t MaxOutput;				
  uint32_t IntegralLimit;
};

#endif
