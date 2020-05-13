#include <needle_planner_manager/needle_state_solver.hpp>


NeedleStateSolver::NeedleStateSolver(double sys_t, int num_stps): _sys_t(sys_t), _num_stps(num_stps){}

NeedleStateSolver::~NeedleStateSolver(){}



void NeedleStateSolver::Solve(const Matrix4d& prev_state, const input_type& inputs, Matrix4d& return_state){
  double sim_t=_sys_t/_num_stps;
  double u1=inputs[0];
  double u2=inputs[1];
  Matrix4d Vee1, Vee2;
  Vee1<<0,0,0,0,
    0,0,-2,0,
    0,2,0,1,
    0,0,0,1;
  Vee2<<0,-1,0,0,
    1,0,0,0,
    0,0,0,0,
    0,0,0,1;
  Matrix4d w1,w2,w3,w4;
  return_state=prev_state;
  for(size_t i=0; i<_num_stps; i++){
    w1=(return_state*Vee1)*u1+(return_state*Vee2)*u2;
    w2=(return_state*Vee1+0.5*sim_t*w1)*u1+(return_state*Vee2+0.5*sim_t*w1)*u2;
    w3=(return_state*Vee1+0.5*sim_t*w2)*u1+(return_state*Vee2+0.5*sim_t*w2)*u2;
    w4=(return_state*Vee1+sim_t*w3)*u1+(return_state*Vee2+sim_t*w3)*u2;
    return_state=return_state+sim_t/6*(w1+w2+w3+w4);
  }    
}


void NeedleStateSolver::Interpolate(const Matrix4d& prev_state, const input_type& inputs, std::vector<Matrix4d>& return_states){
  Matrix4d return_state=prev_state;
  double sim_t=_sys_t/_num_stps;
  double u1=inputs[0];
  double u2=inputs[1];
  Matrix4d Vee1, Vee2;
  Vee1<<0,0,0,0,
      0,0,-2,0,
      0,2,0,1,
      0,0,0,1;
  Vee2<<0,-1,0,0,
    1,0,0,0,
    0,0,0,0,
    0,0,0,1;
  Matrix4d w1,w2,w3,w4;
  return_state=prev_state;
  for(size_t i=0; i<_num_stps; i++){
    w1=(return_state*Vee1)*u1+(return_state*Vee2)*u2;
    w2=(return_state*Vee1+0.5*sim_t*w1)*u1+(return_state*Vee2+0.5*sim_t*w1)*u2;
    w3=(return_state*Vee1+0.5*sim_t*w2)*u1+(return_state*Vee2+0.5*sim_t*w2)*u2;
    w4=(return_state*Vee1+sim_t*w3)*u1+(return_state*Vee2+sim_t*w3)*u2;
    return_state=return_state+sim_t/6*(w1+w2+w3+w4);
    return_states.push_back(return_state);
  }
 }
