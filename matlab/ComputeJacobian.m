function [J_e, T_se] = ComputeJacobian(phi, base_x, base_y, x, T_b0, M_0e, Blist, H_p)
    T_sb = [cos(phi), -sin(phi), 0, base_x;
            sin(phi),  cos(phi), 0, base_y;
                 0  ,       0  , 1, 0.0963;
                 0  ,       0  , 0, 1];
    
    T_se = T_sb * T_b0 * FKinBody(M_0e, Blist, x);
    J_arm = JacobianBody(Blist, x);
    J_base = Adjoint(TransInv(T_se) * T_sb) * [zeros(2,4); H_p; zeros(1,4)];
    J_e = [J_base, J_arm];
end
