function state=motor(state,ctrl,dt,params)
% http://www.profjrwhite.com/system_dynamics/sdyn/s6/s6fmathm/s6fmathm.html

  kt = params.tc; % torque constant
  J  = params.ri; % rotor inertia
  D  = params.dc; % damping coefficient
  Ra = params.er; % motor equivalent resistance Ohm
  La = params.ei; % motor equivalent inductance H
  kv = params.vc; % velocity constant
  i_sig = params.cn; % current noise std A
  w_sig = params.vn; % velocity noise std Hz

  A=[[-Ra/La -kv/La 0];[kt/J -D/J 0];[0 1 0]];
  B=[[1/La 0];[0 -1/J];[0 0]];
  Q=diag([i_sig,w_sig,0])*sqrt(dt);

  d_state=A*state*dt+B*ctrl*dt+Q*randn(3,1);
  state=state+d_state;
