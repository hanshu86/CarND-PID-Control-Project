#ifndef PID_H
#define PID_H

#include <iostream>
#include <vector>

using std::vector;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, bool do_twiddle=false);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   * @return 1 - reset car, 0 - keep running car, 2 - converged
   */
  int UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  double GetSteering(double cte);

 private:

  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * CTE variable
   */
  double prev_cte;
  double integral_cte;
  bool is_prev_cte_valid;

  vector<double> change_factors;
  vector<double> parameters;
  double best_err;
  double total_error;
  bool do_twiddle;
  bool inc_idx;
  int steps_before_eval;
  int steps_for_eval;
  int curr_step_cnt;
  double tolerance;
  int idx;
};

#endif  // PID_H