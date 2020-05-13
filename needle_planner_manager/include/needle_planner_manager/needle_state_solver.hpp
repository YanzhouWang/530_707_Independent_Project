#ifndef NEEDLE_STATE_SOLVER_H
#define NEEDLE_STATE_SOLVER_H


#include <eigen3/Eigen>
#include <eigen3/Dense>


class NeedleStateSolver{
  typedef Eigen::Matrix<double,4,4> Matrix4d;
  typedef std::vector<double> input_type;
private:
  double _sys_t;
  int _num_stps;
public:
  NeedleStateSolver(double sys_t, int num_stps);
  ~NeedleStateSolver();
  void Solve(const Matrix4d& prev_state, const input_type& inputs, Matrix4d& return_state);
  void Interpolate(const Matrix4d& prev_state, const input_type& inputs, std::vector<Matrix4d>& return_states);
};

#endif
