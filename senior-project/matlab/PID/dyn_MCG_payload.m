function [M,C,G] = dyn_MCG_payload(q,dq,P,payload_on)
% Same simplified dynamics structure for joints 2&3 + yaw joint1
% Payload enters via m2eff = m2 + mp when payload_on

if payload_on
    m2eff = P.m2 + P.mp;
else
    m2eff = P.m2;
end

q2=q(2); q3=q(3);
dq2=dq(2); dq3=dq(3);

c3 = cos(q3); s3 = sin(q3);
I2eff = (1/12)*m2eff*P.L2^2;

M11 = P.I1 + I2eff + P.m1*P.lc1^2 + m2eff*(P.L1^2 + P.lc2^2 + 2*P.L1*P.lc2*c3);
M12 = I2eff + m2eff*(P.lc2^2 + P.L1*P.lc2*c3);
M22 = I2eff + m2eff*P.lc2^2;

h = -m2eff*P.L1*P.lc2*s3;

M = [P.J1 0 0;
     0   M11 M12;
     0   M12 M22];

C = [P.b(1) 0 0;
     0 (h*dq3 + P.b(2)) h*(dq2+dq3);
     0 (-h*dq2) (0 + P.b(3))];

% Gravity terms (payload enters explicitly)
G2 = (P.m1*P.lc1 + m2eff*P.L1)*P.g*cos(q2) + m2eff*P.lc2*P.g*cos(q2+q3);
G3 = m2eff*P.lc2*P.g*cos(q2+q3);
G  = [0; G2; G3];
end