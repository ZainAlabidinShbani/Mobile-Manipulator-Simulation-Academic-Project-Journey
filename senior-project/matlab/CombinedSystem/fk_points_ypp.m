function P = fk_points_ypp(q, L)
q1=q(1); q2=q(2); q3=q(3);
L1=L(1); L2=L(2); L3=L(3);

Rz = rotz(q1);
Ry2 = roty(q2);
Ry3 = roty(q3);

p0 = [0;0;0];
p1 = Rz*(Ry2*[L1;0;0]);
p2 = Rz*(Ry2*[L1;0;0] + (Ry2*Ry3)*[L2;0;0]);
p3 = Rz*(Ry2*[L1;0;0] + (Ry2*Ry3)*([L2;0;0] + [L3;0;0]));
P = [p0 p1 p2 p3];
end

