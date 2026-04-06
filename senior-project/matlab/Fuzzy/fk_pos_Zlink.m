function p = fk_pos_Zlink(q,P)
q1=q(1); q2=q(2); q3=q(3);
r = P.L1*cos(q2) + P.L2*cos(q2+q3);
x = cos(q1)*r;
y = sin(q1)*r;
z = P.d1 + P.L1*sin(q2) + P.L2*sin(q2+q3);
p = [x;y;z];
end