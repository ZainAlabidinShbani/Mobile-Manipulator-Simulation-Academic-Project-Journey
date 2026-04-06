function ok = in_bounds(map, ix, iy)
ok = (ix >= 1 && ix <= map.nx && iy >= 1 && iy <= map.ny);
end