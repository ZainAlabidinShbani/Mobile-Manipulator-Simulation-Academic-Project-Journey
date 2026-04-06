function [p,dp,ddp] = cubic_segment(t,T,p0,p1)
a0 = p0; a1 = zeros(3,1);
a2 = 3*(p1-p0)/T^2;
a3 = -2*(p1-p0)/T^3;
p   = a0 + a1*t + a2*t^2 + a3*t^3;
dp  = a1 + 2*a2*t + 3*a3*t^2;
ddp = 2*a2 + 6*a3*t;
end
