function [ranges, ptsW] = lidar_scan_2d(pose, obstacles, apf, lidar)
x = pose(1); y = pose(2); th = pose(3);

angles = linspace(-lidar.fov/2, lidar.fov/2, lidar.nRays) + th;
ranges = lidar.rMax * ones(1, lidar.nRays);

inflate = apf.inflation + apf.safe_margin;

for i = 1:lidar.nRays
    a = angles(i);
    d = [cos(a); sin(a)];
    rHit = lidar.rMax;

    for j = 1:size(obstacles,1)
        c = obstacles(j,1:2)';
        R = obstacles(j,3) + inflate;

        p0 = [x;y];
        m  = p0 - c;

        b = dot(m, d);
        c2 = dot(m,m) - R^2;
        disc = b^2 - c2;

        if disc >= 0
            tt = -b - sqrt(disc);
            if tt > 0 && tt < rHit
                rHit = tt;
            end
        end
    end

    rMeas = rHit + lidar.sigmaR*randn;
    rMeas = max(0.02, min(rMeas, lidar.rMax));
    ranges(i) = rMeas;
end

pts2 = [x; y] + [cos(angles); sin(angles)].*ranges;
ptsW = [pts2; lidar.zHit*ones(1,lidar.nRays)];
end
