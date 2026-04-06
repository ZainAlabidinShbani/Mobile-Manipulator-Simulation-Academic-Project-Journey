function animate_scene_3d_wide_with_map(t, log, obstacles, world, L, cubeSize, map, lidar)
fig = figure('Name','Wide 3D Animation + LiDAR Map'); clf;

ax3 = subplot(1,2,1,'Parent',fig);
set(ax3,'Projection','perspective'); hold(ax3,'on'); grid(ax3,'on');
axis(ax3,'equal');
xlim(ax3, world.xlim); ylim(ax3, world.ylim); zlim(ax3, world.zlim);
xlabel(ax3,'x'); ylabel(ax3,'y'); zlabel(ax3,'z');
title(ax3,'3D Scene');

view(ax3, 45, 22);
campos(ax3, [world.xlim(2)+1.0, world.ylim(1)-1.2, 1.5]);
camtarget(ax3, [(world.xlim(1)+world.xlim(2))/2, (world.ylim(1)+world.ylim(2))/2, 0.2]);
camva(ax3, 9);
lighting(ax3,'gouraud'); camlight(ax3,'headlight');

[Xf,Yf] = meshgrid(linspace(world.xlim(1),world.xlim(2),2), linspace(world.ylim(1),world.ylim(2),2));
surf(ax3, Xf, Yf, zeros(size(Xf)), 'FaceAlpha',0.08, 'EdgeAlpha',0.12);

obsH = 0.12;
for j=1:size(obstacles,1)
    cx=obstacles(j,1); cy=obstacles(j,2); r=obstacles(j,3);
    [Xc,Yc,Zc] = cylinder(r, 28); Zc = Zc*obsH;
    surf(ax3, Xc+cx, Yc+cy, Zc, 'FaceAlpha',0.30, 'EdgeAlpha',0.12);
end

hBaseTrace = plot3(ax3, nan,nan,nan,'LineWidth',1.0);
hEETrace   = plot3(ax3, nan,nan,nan,'LineWidth',1.0);

baseR=0.12; baseH=0.10;
[XB,YB,ZB] = cylinder(baseR, 32); ZB = ZB*baseH;
hBaseSurf = surf(ax3, XB, YB, ZB, 'EdgeAlpha',0.12, 'FaceAlpha',0.35);
hHead = plot3(ax3, [0 0],[0 0],[baseH baseH], 'LineWidth',2);

linkR=0.02; hL1=[]; hL2=[]; hL3=[];
hEE = plot3(ax3, nan,nan,nan,'o','MarkerSize',6,'LineWidth',1.5);

hCube = drawCube(ax3, [0;0;0], cubeSize);

axM = subplot(1,2,2,'Parent',fig);
hold(axM,'on'); axis(axM,'equal'); grid(axM,'on');
xlim(axM, world.xlim); ylim(axM, world.ylim);
title(axM,'LiDAR Occupancy Grid (live)');
xlabel(axM,'x'); ylabel(axM,'y');

P0 = zeros(map.ny, map.nx);
hImg = imagesc(axM, linspace(map.xlim(1),map.xlim(2),map.nx), linspace(map.ylim(1),map.ylim(2),map.ny), P0);
set(axM,'YDir','normal'); colormap(axM, gray);

hPath = plot(axM, nan,nan,'LineWidth',1.2);
hScan = plot(axM, nan,nan,'.','MarkerSize',6);
for j=1:size(obstacles,1)
    draw_circle(axM, obstacles(j,1), obstacles(j,2), obstacles(j,3));
end

skip = max(1, floor(numel(t)/900));

for k = 1:skip:numel(t)
    x  = log.base(k,1); y  = log.base(k,2); th = log.base(k,3);
    q  = log.q(k,:)';
    objw = log.obj(k,:)';

    set(hBaseTrace,'XData',log.base(1:k,1),'YData',log.base(1:k,2),'ZData',zeros(k,1));
    set(hEETrace,  'XData',log.ee(1:k,1),  'YData',log.ee(1:k,2),  'ZData',log.ee(1:k,3));

    c=cos(th); s=sin(th);
    Rz2 = [c -s; s c];
    XY = Rz2 * [XB(:)'; YB(:)'];
    XBr = reshape(XY(1,:), size(XB)) + x;
    YBr = reshape(XY(2,:), size(YB)) + y;
    set(hBaseSurf,'XData',XBr,'YData',YBr,'ZData',ZB);

    headLen=0.18;
    pH = [x; y] + Rz2*[headLen;0];
    set(hHead,'XData',[x pH(1)],'YData',[y pH(2)],'ZData',[baseH baseH]);

    Pp = fk_points_ypp(q, L);
    Rz = rotz(th);
    W0 = [x;y;0] + Rz*Pp(:,1);
    W1 = [x;y;0] + Rz*Pp(:,2);
    W2 = [x;y;0] + Rz*Pp(:,3);
    W3 = [x;y;0] + Rz*Pp(:,4);

    set(hEE,'XData',W3(1),'YData',W3(2),'ZData',W3(3));
    hL1 = updateLinkCylinder(ax3, hL1, W0, W1, linkR);
    hL2 = updateLinkCylinder(ax3, hL2, W1, W2, linkR);
    hL3 = updateLinkCylinder(ax3, hL3, W2, W3, linkR);

    setCubePose(hCube, objw, cubeSize);

    if ~isempty(log.mapP{k})
        set(hImg,'CData',log.mapP{k});
    end
    set(hPath,'XData',log.base(1:k,1),'YData',log.base(1:k,2));

    if ~isempty(log.lidar{k})
        pts = log.lidar{k};
        set(hScan,'XData',pts(1,:),'YData',pts(2,:));
    end

    drawnow;
end
end