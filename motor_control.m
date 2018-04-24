

params = [1000,  % torque constant
          0.5,   % rotor inertia
          0.1,   % damping coefficient
          12,    % motor equivalent resistance Ohm
          0.01,  % motor equivalent inductance H
          0.01,  % velocity constant
          0.001, % current noise
          0.001];% velocity noise
Tl = 0.0; % load torque
cal_err = 1e-4; % set to 0 for perfect cal
tick_err = 1e-6; % encoder tick edge error, sec

NUM_TICKS = 32;
fpga_clock = 40e6;
set_speed_hz = 60;
stable_threshold = 300;

% randomly generate the true encoder spacing
truth_encoder = [1.3 ones(1,NUM_TICKS-1)]+randn(1,NUM_TICKS)*0.01;
truth_encoder = truth_encoder/sum(truth_encoder);
truth_encoder_rad = truth_encoder*2*pi;
% calibrated is truth plus some error
cal_encoder = truth_encoder+randn(size(truth_encoder))*cal_err;
cal_encoder = cal_encoder/sum(cal_encoder);
cal_encoder_rad = cal_encoder*2*pi;

% calibrated encoder in clocks
cal_ticks = cal_encoder*fpga_clock/set_speed_hz;

% simulation timestep
dt = 1e-4;
t = 0;

% speed control initial state
coarse = 2047;
fine = 2047;

tick = 0; % flag indicating we just saw a tick, or 0 for invalid
last_tick_time = 0; % measured time of the last tick
last_tick_time_true = 0; % true time of the last tick
next_tick_idx = 1; % index of the next tick
next_tick_rad = truth_encoder_rad(next_tick_idx); % width of the next tick
motor_state = [0;0;0]; % current A, angular rate rad/s, angular position rad
fsm_state = 0;

kPID_fll = [1e-3;0;0];
kPID_pll = [1e-2;1e-6;1e-2];
ePID = [0;0;0];

state_hist = zeros(floor(10/dt),3+1); % current, rate, pos
state_hist_idx = 1;
tick_hist = zeros(10*50*36,2+1); % truth, meas
tick_hist_idx = 1;
pid_hist = zeros(size(tick_hist),3+1); % P, I, D
pid_hist_idx = 1;
pwm_hist = zeros(size(tick_hist),2+1); % coarse, fine
pwm_hist_idx = 1;

next_t_print = 0;
while t < 10
  % propagate current time
  t = t+dt;
  if t >= next_t_print
    fprintf('t=%.0f\n',t);
    next_t_print = next_t_print+1;
  end
  % current time in units of fpga clocks
  t_clk = t*fpga_clock;
  ctrl = [pwm2V(coarse,fine);Tl]; % voltage V, mechanical load
  % propagate motor state
  motor_state = motor(motor_state,ctrl,dt,params);

  % store history
  state_hist(state_hist_idx,:) = [motor_state' t];
  state_hist_idx+=1;
  pwm_hist(pwm_hist_idx,:) = [coarse fine t];
  pwm_hist_idx += 1;
  % allocate more storage if necessary
  if state_hist_idx >= length(state_hist)
    state_hist(state_hist_idx*2,:) = 0;
  end
  if pwm_hist_idx >= length(pwm_hist)
    pwm_hist(pwm_hist_idx*2,:) = 0;
  end

  % if we've rotated through the next tick
  if motor_state(3) >= next_tick_rad
    % log the tick we just passed
    tick = next_tick_idx;
    this_tick_rad = next_tick_rad;
    % next tick
    next_tick_idx += 1;
    if next_tick_idx > length(cal_encoder_rad)
      next_tick_idx = 1;
    end
    % compute next tick position
    next_tick_rad = next_tick_rad + cal_encoder_rad(next_tick_idx);

    % compute time difference between now and when the tick actually happened
    tick_age = (motor_state(3)-this_tick_rad)/motor_state(2);
    
    tick_time_true = t-tick_age;
    tick_time = tick_time_true+randn(1)*tick_err;
    t2t_time_true = tick_time_true-last_tick_time_true;
    t2t_time = tick_time-last_tick_time;

    tick_hist(tick_hist_idx,:) = [[t2t_time_true t2t_time]*fpga_clock t];
    tick_hist_idx += 1;
    if tick_hist_idx >= length(tick_hist)
      tick_hist(tick_hist_idx*2,:) = 0;
    end
  end
  
  % state machine
  switch fsm_state
    case 0 %accelerate
      % wait two seconds in accelerate
      if t >= 2
        fsm_state = 1;
        stable_cnt = 0;
        first_fll = true;
      end
    case 1 %fll
      if tick
        % error signal is difference between the measured t2t and what it should be
        err = cal_ticks(tick)-t2t_time*fpga_clock;
        if abs(err) < stable_threshold
          stable_cnt += 1;
        else
          stable_cnt = 0;
        end
        ePID = [[0 0 0];[0 1 0];[-1 0 0]]*ePID+[1;1;1]*err;
        if first_fll
          first_fll = false;
          ePID(3) = 0;
        end
        sig = kPID_fll'*ePID;
        coarse = coarse-sig;
        
        pid_hist(pid_hist_idx,:) = [ePID' t];
        pid_hist_idx += 1;
        if pid_hist_idx >= length(pid_hist)
          pid_hist(pid_hist_idx*2,:) = 0;
        end
        
        if stable_cnt >= 1000
          fsm_state = 2;
          % latch the phase error when we switch to pll, want to maintain this
          % phase error is: measured phase of this tick - desired phase of this tick
          latched_phase = this_tick_rad-tick_time
        end
      end
    case 2 %pll
      if tick
        % error signal is difference between the measured tick time and what it should be
      end
  end

  if tick
    % reset the tick flag and wait for a new one
    tick = 0;
    last_tick_time = tick_time;
    last_tick_time_true = tick_time_true;
  end

end

% truncate unused history space
state_hist = state_hist(1:state_hist_idx-1,:);
tick_hist = tick_hist(1:tick_hist_idx-1,:);
pid_hist = pid_hist(1:pid_hist_idx-1,:);
pwm_hist = pwm_hist(1:pwm_hist_idx-1,:);

figure;
% plot spin rate vs time
plot(state_hist(:,end),state_hist(:,2)/2/pi);ylabel('Hz');
title('spin rate');

figure;
% plot tick data vs time, starting with second spin
plot(tick_hist(NUM_TICKS+1:end,end),tick_hist(NUM_TICKS+1:end,1),tick_hist(NUM_TICKS+1:end,end),tick_hist(NUM_TICKS+1:end,2));
title('encoder ticks');

figure;
% plot P term vs time
subplot(311);
plot(pid_hist(:,end),pid_hist(:,1));
title('P');
plot(pid_hist(:,end),pid_hist(:,2));
title('I');
plot(pid_hist(:,end),pid_hist(:,3));
title('D');

figure;
subplot(211);
plot(state_hist(:,end),pwm_hist(:,1));ylabel('coarse');
subplot(212);
plot(pwm_hist(:,end),pwm_hist(:,2));ylabel('fine');
