function plot_case_results(CASE, p_start, p_pick, p_place)
log = CASE.log;

% Path
figure('Color','w'); hold on; grid on; axis equal;
plot3(log.pd(1,:),log.pd(2,:),log.pd(3,:),'k--','LineWidth',1.6);
plot3(log.p(1,:), log.p(2,:), log.p(3,:), 'b','LineWidth',1.6);
plot3(p_start(1),p_start(2),p_start(3),'bo','MarkerSize',8,'MarkerFaceColor','b');
plot3(p_pick(1), p_pick(2), p_pick(3), 'go','MarkerSize',8,'MarkerFaceColor','g');
plot3(p_place(1),p_place(2),p_place(3),'mo','MarkerSize',8,'MarkerFaceColor','m');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('Desired Path','Actual Path','Start','Pick','Place','Location','best');
title(sprintf('Path | %s', CASE.name),'Interpreter','none');

% Errors
figure('Color','w');
subplot(3,1,1); plot(log.t,log.e(1,:),'LineWidth',1.2); grid on; ylabel('e_x (m)');
subplot(3,1,2); plot(log.t,log.e(2,:),'LineWidth',1.2); grid on; ylabel('e_y (m)');
subplot(3,1,3); plot(log.t,log.e(3,:),'LineWidth',1.2); grid on; ylabel('e_z (m)'); xlabel('Time (s)');
my_sgtitle(sprintf('Task-Space Errors | %s', CASE.name));

% Torques
figure('Color','w');
subplot(3,1,1); plot(log.t,log.tau(1,:),'LineWidth',1.1); grid on; ylabel('\tau_1 (N.m)');
subplot(3,1,2); plot(log.t,log.tau(2,:),'LineWidth',1.1); grid on; ylabel('\tau_2 (N.m)');
subplot(3,1,3); plot(log.t,log.tau(3,:),'LineWidth',1.1); grid on; ylabel('\tau_3 (N.m)'); xlabel('Time (s)');
my_sgtitle(sprintf('Joint Torques | %s', CASE.name));
end