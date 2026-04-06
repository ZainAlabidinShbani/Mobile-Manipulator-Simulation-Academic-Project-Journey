function h = updateLinkCylinder(ax, h, P1, P2, r)
n = 16;
[vX,vY,vZ] = cylinder(r, n);
d = P2 - P1; L = norm(d);
if L < 1e-9, L=1e-9; d=[0;0;1]; end
d = d/L;
vZ = vZ*L;

zAxis = [0;0;1];
if norm(cross(zAxis,d)) < 1e-9
    R = eye(3);
    if dot(zAxis,d) < 0, R = diag([1 -1 -1]); end
else
    v = cross(zAxis,d); s = norm(v); c = dot(zAxis,d);
    vx = [  0   -v(3)  v(2);
          v(3)   0    -v(1);
         -v(2)  v(1)   0 ];
    R = eye(3) + vx + vx*vx*((1-c)/(s^2));
end

pts = R * [vX(:)'; vY(:)'; vZ(:)'];
X = reshape(pts(1,:), size(vX)) + P1(1);
Y = reshape(pts(2,:), size(vY)) + P1(2);
Z = reshape(pts(3,:), size(vZ)) + P1(3);

if isempty(h) || ~isvalid(h)
    h = surf(ax, X, Y, Z, 'EdgeAlpha',0.12, 'FaceAlpha',0.80);
else
    set(h,'XData',X,'YData',Y,'ZData',Z);
end
end