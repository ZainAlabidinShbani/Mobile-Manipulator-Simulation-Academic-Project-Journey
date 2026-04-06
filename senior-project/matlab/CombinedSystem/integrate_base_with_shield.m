function [base, vx, vy] = integrate_base_with_shield(base, vx, vy, w, dt, obstacles, apf)
p  = [base.x; base.y];
v  = [vx; vy];

inflate = apf.inflation + apf.safe_margin;
p_next = p + v*dt;

for iter = 1:3
    [inside, ~, ~, dir] = is_inside_any(p_next, obstacles, inflate);
    if ~inside, break; end

    n = dir;
    vn = dot(v, n);
    if vn < 0
        v = v - vn*n;
    end

    tdir = [-n(2); n(1)];
    v = v + 0.15 * tdir;

    p_next = p + v*dt;

    [inside2, j2, ~, dir2] = is_inside_any(p_next, obstacles, inflate);
    if inside2
        c = obstacles(j2,1:2)';
        r = obstacles(j2,3) + inflate;
        eps_out = 1e-3;
        p_next = c + (r + eps_out) * dir2;
        v = v - min(0, dot(v,dir2)) * dir2;
    end
end

base.x  = p_next(1);
base.y  = p_next(2);
base.th = wrapToPi(base.th + w*dt);

vx = v(1); vy = v(2);
end