function h = draw_circle(ax, cx, cy, r)
try
    h = viscircles(ax, [cx cy], r, 'LineWidth', 1.2);
catch
    th = linspace(0,2*pi,100);
    h = plot(ax, cx + r*cos(th), cy + r*sin(th), 'LineWidth', 1.2);
end
end