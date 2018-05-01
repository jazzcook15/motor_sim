function mp=motor_params(tc,ri,dc,er,ei,vc,cn,vn)
mp = struct('tc',tc,... torque constant
            'ri',ri,... rotor inertia
            'dc',dc,... damping coefficient
            'er',er,... equivalent resistance [Ohm]
            'ei',ei,... equivalent inductance [H]
            'vc',vc,... velocity constant
            'cn',cn,... current noise
            'vn',vn); % velocity noise
return

