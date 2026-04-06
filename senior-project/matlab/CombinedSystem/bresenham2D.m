function cells = bresenham2D(x0, y0, x1, y1)
dx = abs(x1-x0);
dy = abs(y1-y0);
sx = sign(x1-x0); if sx==0, sx=1; end
sy = sign(y1-y0); if sy==0, sy=1; end
err = dx - dy;

x = x0; y = y0;
cells = zeros(dx+dy+1,2);
n = 0;

while true
    n = n + 1;
    cells(n,:) = [x y];
    if x==x1 && y==y1, break; end
    e2 = 2*err;
    if e2 > -dy
        err = err - dy;
        x = x + sx;
    end
    if e2 < dx
        err = err + dx;
        y = y + sy;
    end
    if n > 20000, break; end
end

cells = cells(1:n,:);
end