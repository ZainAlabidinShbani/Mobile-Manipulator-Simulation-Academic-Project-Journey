function q_des = ik_3r_ypp_dls(q, p_des, L, lambda, iters, dq_max)
q_des = q;
for k=1:iters
    p = fk_3r_ypp(q_des, L);
    e = p_des - p;
    J = jacobian_num(q_des, L);

    A = (J*J' + (lambda^2)*eye(3));
    dq = J' * (A \ e);

    dq = max(min(dq, dq_max), -dq_max);
    q_des = wrapToPi(q_des + dq);

    if norm(e) < 1e-3
        break;
    end
end
end
