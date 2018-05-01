function [hist_log,sim]=motor_control()

mp = motor_params(1000,   % torque constant
                  0.5,    % rotor inertia
                  0.1,    % damping coefficient
                  12,     % motor equivalent resistance Ohm
                  0.01,   % motor equivalent inductance H
                  0.01,   % velocity constant
                  0.001,  % current noise
                  0.001); % velocity noise
ep = encoder_params(1e-4, % calibration error, set to 0 for perfect cal
                    1e-6, % encoder tick edge error, sec
                    32);  % number of ticks

fpga_clock = 40e6;
set_speed_hz = 60;
stable_threshold = 300;

ctrl = init_ctrl(set_speed_hz,      %
                 stable_threshold,  %
                 [1e-3;0;0],
                 %[3e-2;5e-5;1.5e0], % coarse PID gains. Ku = 3e-2, Tu = 0.35 -> Kp = 0.6Ku = 1.8e-2; Ki = 1.2Ku/Tu = 0.1 -> 5e-5; Kd = 3KuTu/40 = 7.9e-4 -> 1.5
                 [1e-2;1e-6;1e-2],  % fine PID gains
                 0.0);              % load torque

sim = init_sim(5e-4,       % simulation timestep, sec
               5,          % simulation duration, sec
               mp,         % motor parameters
               ep,         % encoder parameters
               ctrl,       % control
               fpga_clock);% fpga clock rate, Hz

% calibrated encoder in clocks
%cal_ticks = cal_encoder*fpga_clock/set_speed_hz;

hist_log = struct('state',zeros(floor(sim.end_t/sim.dt),3+1),... [current, rate, pos]
                  'state_idx',1,...
                  'tick',zeros(floor(sim.end_t*sim.ctrl.set_speed_hz*sim.ep.num_ticks),2+1),... [truth, meas]
                  'tick_idx',1,...
                  'pid',zeros(floor(sim.end_t*sim.ctrl.set_speed_hz*sim.ep.num_ticks),3+1),... [P, I, D]
                  'pid_idx',1,...
                  'pwm',zeros(floor(sim.end_t*sim.ctrl.set_speed_hz*sim.ep.num_ticks),2+1),... [coarse, fine]
                  'pwm_idx',1,
                  'ctrl',zeros(floor(sim.end_t*sim.ctrl.set_speed_hz),2+1),... [speed, ??]
                  'ctrl_idx',1);

next_t_print = 0; % next time to print sim status
while sim.t < sim.end_t

  sim = prop_sim(sim);
  if sim.t >= next_t_print
    fprintf('t=%.1f\n',sim.t);
    next_t_print = next_t_print+0.5;
    fflush(stdout);
  end

  sim = ctrl_sim(sim);
  
  if sim.ctrl.fsm_state >= 1 && sim.ctrl.tick_buf_idx == 0 && sim.tick
    t = sum(sim.ctrl.tick_buf);
    if t > 0
      hist_log.ctrl(hist_log.ctrl_idx,1) = sim.fpga_clk / sum(sim.ctrl.tick_buf);
      hist_log.ctrl(hist_log.ctrl_idx,end) = sim.t;
      hist_log.ctrl_idx += 1;
      if hist_log.ctrl_idx >= length(hist_log.ctrl)
        hist_log.ctrl(hist_log.ctrl_idx*2,:) = 0;
      end
    end
  end

  % store history
  hist_log.state(hist_log.state_idx,:) = [sim.motor_state' sim.t];
  hist_log.state_idx+=1;
  % allocate more storage if necessary
  if hist_log.state_idx >= length(hist_log.state)
    hist_log.state(hist_log.state_idx*2,:) = 0;
  end
  if sim.tick ~= 0
    hist_log.tick(hist_log.tick_idx,:) = [[sim.t2t_time_true sim.t2t_time]*sim.fpga_clk sim.t];
    hist_log.tick_idx += 1;
    hist_log.pid(hist_log.pid_idx,:) = [sim.ctrl.ePID' sim.t];
    hist_log.pid_idx += 1;
    hist_log.pwm(hist_log.pwm_idx,:) = [sim.ctrl.coarse sim.ctrl.fine sim.t];
    hist_log.pwm_idx += 1;
    if hist_log.tick_idx >= length(hist_log.tick)
      hist_log.tick(hist_log.tick_idx*2,:) = 0;
    end
    if hist_log.pid_idx >= length(hist_log.pid)
      hist_log.pid(hist_log.pid_idx*2,:) = 0;
    end
    if hist_log.pwm_idx >= length(hist_log.pwm)
      hist_log.pwm(hist_log.pwm_idx*2,:) = 0;
    end
  end
end

% truncate unused history space
hist_log.state = hist_log.state(1:hist_log.state_idx-1,:);
hist_log.tick  = hist_log.tick(1:hist_log.tick_idx-1,:);
hist_log.pid   = hist_log.pid(1:hist_log.pid_idx-1,:);
hist_log.pwm   = hist_log.pwm(1:hist_log.pwm_idx-1,:);
hist_log.ctrl  = hist_log.ctrl(1:hist_log.ctrl_idx-1,:);

figure;
% plot spin rate vs time
subplot(211);
plot(hist_log.state(:,end),hist_log.state(:,2)/2/pi);ylabel('Hz');
title('spin rate');
% plot current vs time
subplot(212);
plot(hist_log.state(:,end),hist_log.state(:,1)*1e3);ylabel('mA');
title('motor current');

figure;
% plot tick data vs time, starting with second spin
plot(hist_log.tick(sim.ep.num_ticks+1:end,end),hist_log.tick(sim.ep.num_ticks+1:end,1),hist_log.tick(sim.ep.num_ticks+1:end,end),hist_log.tick(sim.ep.num_ticks+1:end,2));
title('encoder ticks');

figure;
% plot P term vs time
subplot(311);
plot(hist_log.pid(:,end),hist_log.pid(:,1));
title('P');
subplot(312);
plot(hist_log.pid(:,end),hist_log.pid(:,2));
title('I');
subplot(313);
plot(hist_log.pid(:,end),hist_log.pid(:,3));
title('D');

figure;
subplot(211);
plot(hist_log.pwm(:,end),hist_log.pwm(:,1));ylabel('coarse');
subplot(212);
plot(hist_log.pwm(:,end),hist_log.pwm(:,2));ylabel('fine');

figure;
plot(hist_log.state(:,end),hist_log.state(:,2)/2/pi,hist_log.ctrl(:,end),hist_log.ctrl(:,1));ylabel('Hz');
legend('true speed', 'measured speed');
return


function ctrl=init_ctrl(set_speed_hz,
                        stable_threshold,
                        kPID_fll,
                        kPID_pll,
                        load_torque)
ctrl = struct('set_speed_hz',set_speed_hz,...
              'stable_threshold',stable_threshold,...
              'coarse',2047,....
              'fine',2047,...
              'Tl',load_torque,...
              'kPID_fll',kPID_fll,...
              'kPID_pll',kPID_pll,...
              'ePID',[0;0;0],...
              'fsm_state',0,...
              'accel_sec',2,...
              'fine_threshold',1000,...
              'tick_buf',[],...
              'tick_buf_idx',0,...
              'cal_tick_clk',[]); % expected encoder ticks in fpga clocks
return

function sim=init_sim(dt,end_t,mp,ep,ctrl,fc)
sim = struct('dt',dt,...                   simulation timestep
             'end_t',end_t,...             simulation end time
             'mp',mp,...                   motor parameters
             'ep',ep,...                   encoder parameters
             'ctrl',ctrl,...               control structure
             'fpga_clk',fc,...             fpga clock rate
             'truth_ticks',[],...          true tick locations
             'truth_ticks_rad',[],...      same as above, but in radians
             'cal_ticks',[],...            simulated calibration tick locations
             'cal_ticks_rad',[],...        same as above, but in radians
             't',0,...                     current simulation time
             't_clk',0,...                 current sumlation time in units of fpga clocks
             'tick', 0,...                 index of tick we just saw, or 0 for invalid
             'last_tick_time', 0,...       measured time of the last tick
             'last_tick_time_true', 0,...  true time of the last tick
             'next_tick_idx', 1,...        index of the next tick
             'next_tick_rad', 0,...        width of the next tick
             'motor_state', [0;0;0]);    % current [A], angular rate [rad/s], angular position [rad]

% randomly generate the true encoder spacing
sim.truth_ticks = [1.3 ones(1, ep.num_ticks - 1)] + randn(1, ep.num_ticks) * 0.01;
sim.truth_ticks = sim.truth_ticks / sum(sim.truth_ticks);
sim.truth_ticks_rad = sim.truth_ticks*2*pi;
% calibrated is truth plus some error
sim.cal_ticks = sim.truth_ticks + randn(size(sim.truth_ticks)) * ep.ce;
sim.cal_ticks = sim.cal_ticks / sum(sim.cal_ticks);
sim.cal_ticks_rad = sim.cal_ticks*2*pi;

sim.next_tick_rad = sim.truth_ticks_rad(sim.next_tick_idx);

sim.ctrl.cal_tick_clk = sim.cal_ticks * sim.fpga_clk / sim.ctrl.set_speed_hz;
return

function sim=prop_sim(sim)

if sim.tick
  % reset the tick flag and wait for a new one
  sim.tick = 0;
  sim.last_tick_time = sim.tick_time;
  sim.last_tick_time_true = sim.tick_time_true;
end

% propagate current time
sim.t = sim.t+sim.dt;
% current time in units of fpga clocks
sim.t_clk = sim.t*sim.fpga_clk;
ctrl = [pwm2V(sim.ctrl.coarse,sim.ctrl.fine);sim.ctrl.Tl]; % voltage V, mechanical load
% propagate motor state
sim.motor_state = motor(sim.motor_state, ctrl, sim.dt, sim.mp);

% if we've rotated through the next tick
if sim.motor_state(3) >= sim.next_tick_rad
  % log the tick we just passed
  sim.tick = sim.next_tick_idx;
  sim.this_tick_rad = sim.next_tick_rad;
  % next tick
  sim.next_tick_idx += 1;
  if sim.next_tick_idx > length(sim.cal_ticks_rad)
    sim.next_tick_idx = 1;
  end
  % compute next tick position
  sim.next_tick_rad = sim.next_tick_rad + sim.cal_ticks_rad(sim.next_tick_idx);

  % compute time difference between now and when the tick actually happened
  sim.tick_age = (sim.motor_state(3)-sim.this_tick_rad)/sim.motor_state(2);
  
  sim.tick_time_true = sim.t-sim.tick_age;
  sim.tick_time_true_clk = sim.tick_time_true * sim.fpga_clk;
  sim.tick_time = sim.tick_time_true+randn(1)*sim.ep.te;
  sim.tick_time_clk = sim.tick_time * sim.fpga_clk;
  sim.t2t_time_true = sim.tick_time_true-sim.last_tick_time_true;
  sim.t2t_time_true_clk = sim.t2t_time_true * sim.fpga_clk;
  sim.t2t_time = sim.tick_time-sim.last_tick_time;
  sim.t2t_time_clk = sim.t2t_time * sim.fpga_clk;
end

return


function sim=ctrl_sim(sim)
% state machine
switch sim.ctrl.fsm_state
  case 0 %accelerate
    % wait two seconds in accelerate
    if sim.t >= sim.ctrl.accel_sec
      sim.ctrl.fsm_state = 1;
      sim.ctrl.stable_cnt = 0;
      sim.ctrl.first_fll = true;
    end
  case 1 %fll
    if sim.tick
      sim.ctrl.tick_buf(sim.ctrl.tick_buf_idx + 1) = sim.t2t_time_clk;
      sim.ctrl.tick_buf_idx += 1;
      if sim.ctrl.tick_buf_idx >= sim.ep.num_ticks
        sim.ctrl.tick_buf_idx = 0;
      end
      % error signal is difference between the measured t2t and what it should be
      err = sim.ctrl.cal_tick_clk(sim.tick)-sim.t2t_time_clk;
      %fprintf('%.0f %.0f\n', sim.ctrl.cal_tick_clk(sim.tick), sim.t2t_time_clk);
      if abs(err) < sim.ctrl.stable_threshold
        sim.ctrl.stable_cnt += 1;
      else
        sim.ctrl.stable_cnt = 0;
      end
      sim.ctrl.ePID = [[0 0 0];[0 1 0];[-1 0 0]]*sim.ctrl.ePID+[1;1;1]*err;
      if sim.ctrl.first_fll
        sim.ctrl.first_fll = false;
        sim.ctrl.ePID(3) = 0;
      end
      sig = sim.ctrl.kPID_fll'*sim.ctrl.ePID;
      sim.ctrl.coarse = sim.ctrl.coarse-sig;
      if sim.ctrl.coarse > 4095
        sim.ctrl.coarse = 4095;
      end
      if sim.ctrl.coarse < 0
        sim.ctrl.coarse = 0;
      end

      if sim.ctrl.stable_cnt >= sim.ctrl.fine_threshold
        sim.ctrl.fsm_state = 2;
        % latch the phase error when we switch to pll, want to maintain this
        % phase error is: measured phase of this tick - desired phase of this tick
        sim.ctrl.latched_phase = sim.this_tick_rad-sim.tick_time;
      end
    end
  case 2 %pll
    if sim.tick
      % error signal is difference between the measured tick time and what it should be
    end
end
return
