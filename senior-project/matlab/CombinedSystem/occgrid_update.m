function map = occgrid_update(map, pose, ranges, lidar)
x = pose(1); y = pose(2); th = pose(3);
angles = linspace(-lidar.fov/2, lidar.fov/2, lidar.nRays) + th;

for i = 1:lidar.nRays
    a = angles(i);
    r = ranges(i);

    x1 = x + r*cos(a);
    y1 = y + r*sin(a);

    [ix0, iy0] = world_to_grid(map, x,  y);
    [ix1, iy1] = world_to_grid(map, x1, y1);

    if ~in_bounds(map, ix0, iy0)
        continue;
    end

    cells = bresenham2D(ix0, iy0, ix1, iy1);
    if isempty(cells), continue; end

    for k = 1:size(cells,1)-1
        ix = cells(k,1); iy = cells(k,2);
        if in_bounds(map, ix, iy)
            map.L(iy,ix) = clamp(map.L(iy,ix) + map.L_free, map.Lmin, map.Lmax);
        end
    end

    if r < (lidar.rMax - 1e-6)
        ix = cells(end,1); iy = cells(end,2);
        if in_bounds(map, ix, iy)
            map.L(iy,ix) = clamp(map.L(iy,ix) + map.L_occ, map.Lmin, map.Lmax);
        end
    end
end
end