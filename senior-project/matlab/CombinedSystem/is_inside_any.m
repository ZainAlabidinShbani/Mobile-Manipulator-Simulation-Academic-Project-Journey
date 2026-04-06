function [inside, jmin, dmin, dir] = is_inside_any(p, obstacles, inflate)
inside = false;
jmin = 1;
dmin = inf;
dir  = [1;0];

for j = 1:size(obstacles,1)
    c = obstacles(j,1:2)';
    r = obstacles(j,3) + inflate;

    dvec = p - c;
    dist = norm(dvec);
    if dist < 1e-9
        dist = 1e-9;
        dvec = [1;0];
    end
    d = dist - r;

    if d < dmin
        dmin = d;
        jmin = j;
        dir  = dvec / dist;
    end

    if d < 0
        inside = true;
    end
end
end
