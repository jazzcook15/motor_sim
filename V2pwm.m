function [coarse,fine]=V2pwm(V)
%{
  b=2.5;
  a=5.55e-4;
  
  % first, assume midpoint fine and compute coarse
  coarse=round((V - b)/a-204.8);

  % clamp coarse to allowable range  
  if coarse < 0
    coarse = 0;
  elseif coarse > 4095
    coarse = 4095;
  end

  % with coarse correctly computed, now compute correct fine
  fine = round(((V - b)/a - coarse) * 10);
  
  % clamp fine to allowable range
  if fine < 0
    fine = 0;
  elseif fine > 4095
    fine = 4095;
  end
%}

  b=2.5;
  a=6.105e-4;
  coarse = ( V - b ) / a;
  fine = 0;
