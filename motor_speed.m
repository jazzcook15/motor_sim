

mp = motor_params(1000,   % torque constant
                  0.5,    % rotor inertia
                  0.1,    % damping coefficient
                  12,     % motor equivalent resistance Ohm
                  0.01,   % motor equivalent inductance H
                  0.01,   % velocity constant
                  0.001,  % current noise
                  0.001); % velocity noise
Tl=0.0; % load torque


dt=1e-4;
t_end=4;
t_vec=0:dt:t_end;
num_t=length(t_vec);

V_vec=[2 3 4 5];

l={};
figure;
for V=V_vec
  state=[0;0;0]; % current A, angular rate rad/s, angular position rad
  ctrl=[V;Tl]; % voltage V, mechanical load

  state_hist=zeros(num_t,length(state));
  hist_idx=1;
  state_hist(hist_idx,1:3)=state';

  for t=t_vec(2:end)
    state=motor(state,ctrl,dt,mp);
    hist_idx=hist_idx+1;
    state_hist(hist_idx,:)=state';
  end
  printf('V=%d f=%.1f\n',V,state(2)/2/pi);
  subplot(211);
  plot(t_vec,state_hist(:,1)*1e3);hold all;grid on;ylabel('mA');
  subplot(212);
  plot(t_vec,state_hist(:,2)/2/pi);hold all;grid on;ylabel('Hz');
  %subplot(313);
  %plot(t_vec,mod(state_hist(:,3)/2/pi*360,360));hold all;grid on;ylabel('deg');
  l{end+1}=sprintf('V=%.1f',V);
end
subplot(211);
legend(l);
