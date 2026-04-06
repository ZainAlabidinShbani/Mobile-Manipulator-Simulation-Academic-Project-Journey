function J = jacobian_pos_Zlink(q,P)
q1=q(1); q2=q(2); q3=q(3);
r   = P.L1*cos(q2) + P.L2*cos(q2+q3);
dr2 = -P.L1*sin(q2) - P.L2*sin(q2+q3);
dr3 = -P.L2*sin(q2+q3);
dz2 =  P.L1*cos(q2) + P.L2*cos(q2+q3);
dz3 =  P.L2*cos(q2+q3);

J = [ -sin(q1)*r,  cos(q1)*dr2,  cos(q1)*dr3;
       cos(q1)*r,  sin(q1)*dr2,  sin(q1)*dr3;
              0 ,        dz2  ,        dz3 ];
end