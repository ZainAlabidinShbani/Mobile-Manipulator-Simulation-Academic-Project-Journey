function [v, dmin] = apf_velocity_robust(p, goal, obstacles, apf)
Fatt = apf.k_att * (goal - p);

Frep = [0;0];
Ftan = [0;0];
dmin = inf;

for j = 1:size(obstacles,1)
    c = obstacles(j,1:2)';
    r = obstacles(j,3) + apf.inflation + apf.safe_margin;

    dvec = p - c;
    dist = norm(dvec);
    if dist < 1e-9, dist = 1e-9; end

    d = dist - r;
    dmin = min(dmin, d);

    if d < apf.d0
        dir = dvec / dist;
        rep = apf.k_rep * ((1/max(d,1e-6) - 1/apf.d0) * (1/max(d,1e-6)^2));
        Frep = Frep + rep * dir;

        tdir = [ -dir(2); dir(1) ];
        toGoal = goal - p;
        if norm(toGoal) > 1e-9
            sgn = sign(det([tdir, toGoal/norm(toGoal)]));
            if sgn == 0, sgn = 1; end
        else
            sgn = 1;
        end
        Ftan = Ftan + apf.swirl_k * rep * sgn * tdir;
    end

    if d < apf.hard_stop_d
        dir = dvec / dist;
        Frep = Frep + 12.0 * dir;
    end
end

F = Fatt + Frep + Ftan;
v = F;

vm = norm(v);
if vm > apf.v_apf_max
    v = v * (apf.v_apf_max / max(vm,1e-9));
end

if norm(Fatt) > 1e-9 && dmin > apf.hard_stop_d && norm(v) < 0.04
    v = 0.04 * (Fatt / norm(Fatt));
end
end
