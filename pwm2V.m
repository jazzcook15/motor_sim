function V=pwm2V(coarse,fine)
%{
  b=2.5;
  a=5.55e-4;
  
  V = a * (coarse + fine/10) + b;
%}

  b=2.5;
  a=6.105e-4;
  V = a * coarse + b;

