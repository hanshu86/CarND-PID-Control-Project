#include <math.h>
#include <numeric>
#include "PID.h"

using namespace std;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {

}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool twiddle) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  prev_cte = 0.0;
  integral_cte = 0.0;
  is_prev_cte_valid = false;

  parameters.push_back(Kp);
  parameters.push_back(Ki);
  parameters.push_back(Kd);
  change_factors.push_back(0.0178849);//Kp
  change_factors.push_back(0.0119725);//Ki
  change_factors.push_back(0.0541994);//Kd
  best_err = 99999999.0;//init with some large value
  do_twiddle = twiddle;
  steps_before_eval = 100;
  steps_for_eval = 1000;
  curr_step_cnt = 0;
  total_error = 0.0;
  tolerance = 0.001;
  idx = 0;
  inc_idx = true;
}

int PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  int rc = 0;//initialinze with running car
  bool check_tolerance = false;

  curr_step_cnt++;
  // cout << "Current step count: " << curr_step_cnt << endl;
  if((curr_step_cnt == 1) && (inc_idx)) {
    parameters[idx] += change_factors[idx];
    cout << "idx : " << idx << endl;
    cout << "Value: " << parameters[idx] << endl;
    cout << "Kp: " << parameters[0] << endl;
    cout << "Ki: " << parameters[1] << endl;
    cout << "Kd: " << parameters[2] << endl;
  }
  else
  {
    cout << "No idx change" << endl;
    cout << "Kp: " << parameters[0] << endl;
    cout << "Ki: " << parameters[1] << endl;
    cout << "Kd: " << parameters[2] << endl;
  }

  if(curr_step_cnt >= steps_before_eval) {
    total_error += pow(cte,2);
  }

  if((do_twiddle) && (curr_step_cnt >= (steps_for_eval+steps_before_eval))) {
    total_error /= steps_for_eval;

    cout << "IN ERROR LOOP" << endl;
    cout << "Kp: " << parameters[0] << endl;
    cout << "Ki: " << parameters[1] << endl;
    cout << "Kd: " << parameters[2] << endl;
    cout << "Kp Fact: " << change_factors[0] << endl;
    cout << "Ki Fact: " << change_factors[1] << endl;
    cout << "Kd Fact: " << change_factors[2] << endl;

    if(inc_idx) {
      if(total_error < best_err ) {
        cout << "Got better error in if case" << endl;
        best_err = total_error;
        change_factors[idx] *= 1.1;
        inc_idx = true;
        idx++;
        if(idx > 2) {
          idx = 0;
          check_tolerance = true;
        }
      }
      else {
        cout << "Need to decrease param" << endl;
        parameters[idx] -= 2 * change_factors[idx];
        inc_idx = false;
      }
    }
    else {
      if(total_error < best_err ) {
        cout << "Got better error in else case" << endl;
        best_err = total_error;
        change_factors[idx] *= 1.1;
      }
      else {
        cout << "change param in else else case" << endl;
        parameters[idx] += change_factors[idx];
        change_factors[idx] *= 0.9;
      }
      inc_idx = true;
      idx++;
      if(idx > 2) {
        idx = 0;
        check_tolerance = true;
      }
    }


    cout << "RESET CAR" << endl;
    cout << "Kp: " << parameters[0] << endl;
    cout << "Ki: " << parameters[1] << endl;
    cout << "Kd: " << parameters[2] << endl;
    cout << "Kp Fact: " << change_factors[0] << endl;
    cout << "Ki Fact: " << change_factors[1] << endl;
    cout << "Kd Fact: " << change_factors[2] << endl;
    cout << "Best Error: " << best_err << endl;
    cout << "Total Error: " << total_error << endl;
    cout << "Sum is: " << accumulate(change_factors.begin(), change_factors.end(), 0.0) << endl;
    rc = 1; //reset car position
    curr_step_cnt = 0;
    total_error = 0;
    is_prev_cte_valid = false;
  }

  if ((check_tolerance) && (accumulate(change_factors.begin(), change_factors.end(), 0.0) <= tolerance)) {
    cout << "Params Converged: " << endl;
    cout << "Kp: " << parameters[0] << endl;
    cout << "Ki: " << parameters[1] << endl;
    cout << "Kd: " << parameters[2] << endl;
    cout << "Kp Fact: " << change_factors[0] << endl;
    cout << "Ki Fact: " << change_factors[1] << endl;
    cout << "Kd Fact: " << change_factors[2] << endl;
    cout << "Sum is: " << accumulate(change_factors.begin(), change_factors.end(), 0.0) << endl;
    rc = 2;//converged
  }

  Kp = parameters[0];
  Ki = parameters[1];
  Kd = parameters[2];

  // cout << "Kp: " << parameters[0] << endl;
  // cout << "Ki: " << parameters[1] << endl;
  // cout << "Kd: " << parameters[2] << endl;

  return rc;
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