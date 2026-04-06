
function plot_joint_profiles_and_manip(log, caseName)
t = log.t;

% 1) Joint angles: actual vs desired
figure('Color','w');
subplot(3,1,1);
plot(t, log.q(1,:), 'LineWidth',1.3); hold on;
plot(t, log.qd(1,:), '--', 'LineWidth',1.3);
grid on; ylabel('q_1 (rad)'); legend('Actual','Desired','Location','best');

subplot(3,1,2);
plot(t, log.q(2,:), 'LineWidth',1.3); hold on;
plot(t, log.qd(2,:), '--', 'LineWidth',1.3);
grid on; ylabel('q_2 (rad)'); legend('Actual','Desired','Location','best');

subplot(3,1,3);
plot(t, log.q(3,:), 'LineWidth',1.3); hold on;
plot(t, log.qd(3,:), '--', 'LineWidth',1.3);
grid on; ylabel('q_3 (rad)'); xlabel('Time (s)'); legend('Actual','Desired','Location','best');

my_sgtitle(['Joint Angle Profiles: Actual vs Desired (', caseName, ')']);

% 2) Joint angle errors (desired - actual)
figure('Color','w');
subplot(3,1,1);
plot(t, log.qerr(1,:), 'LineWidth',1.3); grid on;
ylabel('e_{q1} (rad)');

subplot(3,1,2);
plot(t, log.qerr(2,:), 'LineWidth',1.3); grid on;
ylabel('e_{q2} (rad)');

subplot(3,1,3);
plot(t, log.qerr(3,:), 'LineWidth',1.3); grid on;
ylabel('e_{q3} (rad)'); xlabel('Time (s)');

my_sgtitle(['Joint Angle Tracking Errors (qd - q) (', caseName, ')']);

% 3) Manipulability indicator
figure('Color','w');
plot(t, log.sig, 'LineWidth',1.4);
grid on; xlabel('Time (s)'); ylabel('\sigma_{min}(J)');
title(['Manipulability Indicator \sigma_{min}(J) (', caseName, ')']);
end