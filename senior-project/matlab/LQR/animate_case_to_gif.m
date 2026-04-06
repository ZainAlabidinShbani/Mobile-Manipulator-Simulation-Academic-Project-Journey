function animate_case_to_gif(log, P, p_pick, p_place, cube_size, stepAnim, gifDelay, gifFile, titleStr)
figure('Color','w'); hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45,25);
title(['Animation: ', titleStr]);
xlim([-0.8 0.8]); ylim([-0.8 0.8]); zlim([0 0.7]);

plot3(0,0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');

h_l0 = plot3([0 0],[0 0],[0 0],'b','LineWidth',3);
h_l1 = plot3([0 0],[0 0],[0 0],'r','LineWidth',3);
h_l2 = plot3([0 0],[0 0],[0 0],'g','LineWidth',3);
h_ee = plot3(0,0,0,'ro','MarkerSize',6,'MarkerFaceColor','r');

h_path_act = plot3(NaN,NaN,NaN,'b','LineWidth',1.6);
h_path_des = plot3(NaN,NaN,NaN,'k--','LineWidth',1.2);

h_cube = patch('Faces',[1 2 3 4 5 6 7 8], ...
    'Vertices',cube_vertices(p_pick,cube_size), ...
    'FaceColor',[0.2 0.8 0.3],'EdgeColor','k');

Xa=[]; Ya=[]; Za=[];
Xd=[]; Yd=[]; Zd=[];

isFirstFrame = true;

for k = 1:stepAnim:length(log.t)
    qk = log.q(:,k);
    th1=qk(1); th2=qk(2); th3=qk(3);

    p0 = [0;0;0];
    p1 = [0;0;P.d1];

    r2 = P.L1*cos(th2);
    z2 = P.d1 + P.L1*sin(th2);
    p2 = [cos(th1)*r2; sin(th1)*r2; z2];

    r3 = P.L1*cos(th2) + P.L2*cos(th2+th3);
    z3 = P.d1 + P.L1*sin(th2) + P.L2*sin(th2+th3);
    p3 = [cos(th1)*r3; sin(th1)*r3; z3];

    set(h_l0,'XData',[p0(1) p1(1)],'YData',[p0(2) p1(2)],'ZData',[p0(3) p1(3)]);
    set(h_l1,'XData',[p1(1) p2(1)],'YData',[p1(2) p2(2)],'ZData',[p1(3) p2(3)]);
    set(h_l2,'XData',[p2(1) p3(1)],'YData',[p2(2) p3(2)],'ZData',[p2(3) p3(3)]);
    set(h_ee,'XData',p3(1),'YData',p3(2),'ZData',p3(3));

    if log.pay(k)==1
        set(h_cube,'Vertices',cube_vertices(p3,cube_size));
    else
        if log.seg(k) < 2
            set(h_cube,'Vertices',cube_vertices(p_pick,cube_size));
        else
            set(h_cube,'Vertices',cube_vertices(p_place,cube_size));
        end
    end

    Xa(end+1)=p3(1); Ya(end+1)=p3(2); Za(end+1)=p3(3);
    Xd(end+1)=log.pd(1,k); Yd(end+1)=log.pd(2,k); Zd(end+1)=log.pd(3,k);

    set(h_path_act,'XData',Xa,'YData',Ya,'ZData',Za);
    set(h_path_des,'XData',Xd,'YData',Yd,'ZData',Zd);

    drawnow;

    frame = getframe(gcf);
    img = frame2im(frame);
    [imind, cm] = rgb2ind(img, 256);

    if isFirstFrame
        imwrite(imind, cm, gifFile, 'gif', 'Loopcount', inf, 'DelayTime', gifDelay);
        isFirstFrame = false;
    else
        imwrite(imind, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelay);
    end
end
end