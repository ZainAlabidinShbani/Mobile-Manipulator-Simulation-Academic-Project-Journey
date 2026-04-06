function q = ik_geom_Zlink(p,P,elbow)
if nargin<3, elbow='down'; end
x=p(1); y=p(2); z=p(3);

q1 = atan2(y,x);
r  = sqrt(x^2 + y^2);
zp = z - P.d1;

D = (r^2 + zp^2 - P.L1^2 - P.L2^2)/(2*P.L1*P.L2);
D = max(-1,min(1,D));

if strcmpi(elbow,'down')
    q3 = -acos(D);
else
    q3 =  acos(D);
end

phi = atan2(zp, r);
psi = atan2(P.L2*sin(q3), P.L1 + P.L2*cos(q3));
q2 = phi - psi;

q = [q1;q2;q3];
end