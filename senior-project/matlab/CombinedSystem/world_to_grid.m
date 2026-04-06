function [ix, iy] = world_to_grid(map, x, y)
ix = floor((x - map.xlim(1))/map.res) + 1;
iy = floor((y - map.ylim(1))/map.res) + 1;
end
