function ep=encoder_params(ce,te,nt)
ep = struct('ce',ce,... calibration error, frac of circle
            'te',te,... encoder tick edge error, sec
            'num_ticks',nt); % number of encoder ticks
return

