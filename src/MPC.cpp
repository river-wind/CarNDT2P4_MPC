#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10; //CEL started with 20, which caused chaos.  Same with 15. Q&A recommended 10, which worked well enough.
double dt = 0.1;//CEL 1 second divided by N 

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

//CEL: define all local variables for future use.  Using Q&A naming convention for ease 
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 60;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` is a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    fg[0] = 0;
	  
    // Reference state cost
    //TODO: Define the cost related te reference 
    //CEL: loop through N states and sum everything into the fg[0], the cost function.
    for (size_t i = 0; i < N; i++) {
      //sum the ctes, and multiply by 200 in order to give it a higher wight in the final cost
      fg[0] += 200 * CppAD::pow(vars[cte_start + i] - ref_cte, 2); 
      //add in the psi error, and multiply by 200 in order to give higher weight to it in the cost
      fg[0] += 200 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      //add in the velocity, some weighting to limit the accelleration, which ws going too high initially
      fg[0] += 2 * CppAD::pow(vars[v_start + i] - ref_v, 2);
    }
  
    for (size_t i = 0; i < N - 1; i++) {
      //add in the steering angle change, multiplied by 10 to smooth the steering angle transition
      fg[0] += 10 * CppAD::pow(vars[delta_start + i], 2);
      //add in the current acceleration, multiplied by 10 to smooth the speed changes
      fg[0] += 10 * CppAD::pow(vars[a_start + i], 2);
    }
	  
    for (size_t i = 0; i < N -2; i++) {
      //smooth the differences between sequential steering angle changes
	//multiplied by 150 to smooth the steering angle transition at a higher cost than above
      fg[0] += 150 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2); 
      //smooth the differences between sequential acceleration changes as a cost with lower weight.
      fg[0] += 20 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    // fg[0] holds the above cost function totals, so the constraints start at fg[1]	  
    // Set initial values
    fg[x_start + 1] = vars[x_start];
    fg[y_start + 1] = vars[y_start];
    fg[psi_start + 1] = vars[psi_start];
    fg[v_start + 1] = vars[v_start];
    fg[cte_start + 1] = vars[cte_start];
    fg[epsi_start + 1] = vars[epsi_start];
	  
    //the rest of the constraints, taken mostly from classroom quiz
    for (size_t i = 0; i < N -1; i++) {
      //values at time t+1
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];
		
      //values at time t
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];
  	
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];
 	
      //calculate the polynomial value, target y-position change
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0* x0 * x0;
      //desired psi
      AD<double> psides0 = CppAD::atan(3*coeffs[3] * x0 * x0 + 2*coeffs[2] * x0 + coeffs[1]);
		
      //TODO: Set up the rest of the model constraints
      //classroom model equations
      //Kinematic model equations as equality constraints.  Values will be asserted to 0 for the solver.
      fg[x_start + i + 2] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[y_start + i + 2] = y1 - (y0 + v0 *CppAD::sin(psi0) * dt);
      fg[psi_start + i + 2] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[v_start + i + 2] = v1 - (v0 + a0 * dt);
      fg[cte_start + i + 2] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[epsi_start + i + 2] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
//  size_t i;  //not used in this scope
  typedef CPPAD_TESTVECTOR(double) Dvector;

  //CEL: Local variables to hold state values
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:

  // CEL: n_vars is the number of time steps * the features we're handling(6), plus delta and accelleration
  size_t n_vars = N * 6 + (N - 1) * 2;

  // TODO: Set the number of constraints
  // CEL: 6 equations times the number of time steps
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // CEL: Set all upper and lower vars limits
  // to the max possible values
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
	
  // From the instructions, upper and lower limites of delta are set to -25 and 25
  // degrees (in radians)
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
	
  //Acceleration/decceleration upper and lower limits
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0 * Lf;  
    vars_upperbound[i] = 1.0 * Lf;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  //CEL; set the lower and upper bounds to the initial state 
  //so that the solver has somewhere to start
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
	  
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  //return {};

  //Return object	
  vector<double> retVec;

  //Push onto the object the steering angle and throttle values	
  retVec.push_back(solution.x[delta_start]);
  retVec.push_back(solution.x[a_start]);

  //Push onto the object pairs of solution trajectory points	
  for (size_t i = 0; i < N-1; i++)
  {
    retVec.push_back(solution.x[x_start + i + 1]);
    retVec.push_back(solution.x[y_start + i + 1]);
  }
	
  //Return the result
  return retVec;
}
