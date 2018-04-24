function state=motor(state,ctrl,dt,params)
% http://www.profjrwhite.com/system_dynamics/sdyn/s6/s6fmathm/s6fmathm.html

  kt = params(1); % torque constant
  J  = params(2); % rotor inertia
  D  = params(3); % damping coefficient
  Ra = params(4); % motor equivalent resistance Ohm
  La = params(5); % motor equivalent inductance H
  kv = params(6); % velocity constant
  i_sig = params(7); % current noise std A
  w_sig = params(8); % velocity noise std Hz

  A=[[-Ra/La -kv/La 0];[kt/J -D/J 0];[0 1 0]];
  B=[[1/La 0];[0 -1/J];[0 0]];
  Q=diag([i_sig,w_sig,0])*sqrt(dt);

  d_state=A*state*dt+B*ctrl*dt+Q*randn(3,1);
  state=state+d_state;
