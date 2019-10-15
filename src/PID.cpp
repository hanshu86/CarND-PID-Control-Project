#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {

}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double tolerance) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  tol = tolerance;

  prev_cte = 0.0;
  integral_cte = 0.0;
  is_prev_cte_valid = false;

  params.push_back(Kp);
  params.push_back(Kd);
  params.push_back(Ki);

  change_factors.push_back(1.0);
  change_factors.push_back(1.0);
  change_factors.push_back(1.0);
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte * cte;
  i_error = cte * cte;
  d_error = cte * cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}

double PID::GetSteering(double cte) {
  double steer = 0.0;
  double diff_cte = is_prev_cte_valid ? (cte - prev_cte) : 0;
  prev_cte = cte;
  is_prev_cte_valid = true;
  integral_cte += cte;

  steer = (-1 *  Kp * cte) + (-1 * Kd * diff_cte) + (-1 * Ki * integral_cte);

  return steer;
}