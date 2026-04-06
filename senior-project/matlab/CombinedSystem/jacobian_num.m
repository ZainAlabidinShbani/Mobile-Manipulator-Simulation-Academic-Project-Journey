function J = jacobian_num(q, L)
eps = 1e-6;
p0 = fk_3r_ypp(q, L);
J = zeros(3,3);
for i=1:3
    dq = zeros(3,1); dq(i)=eps;
    p1 = fk_3r_ypp(q + dq, L);
    J(:,i) = (p1 - p0)/eps;
end
end
