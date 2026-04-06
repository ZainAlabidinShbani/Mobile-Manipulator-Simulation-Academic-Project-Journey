clc; clear;

% Total duration
Tf = 5;                      % Total time duration
t = linspace(0, Tf, 100);    % Time vector

% Preallocate arrays
s_cubic = zeros(size(t));
s_quintic = zeros(size(t));
ds_cubic = zeros(size(t));
ds_quintic = zeros(size(t));
dds_cubic = zeros(size(t));
dds_quintic = zeros(size(t));

% Calculate values
for i = 1:length(t)
    tau = t(i)/Tf;
    
    % Cubic
    s_cubic(i) = 3*tau^2 - 2*tau^3;
    ds_cubic(i) = (6*tau - 6*tau^2)/Tf;
    dds_cubic(i) = (6 - 12*tau)/Tf^2;

    % Quintic
    s_quintic(i) = 10*tau^3 - 15*tau^4 + 6*tau^5;
    ds_quintic(i) = (30*tau^2 - 60*tau^3 + 30*tau^4)/Tf;
    dds_quintic(i) = (60*tau - 180*tau^2 + 120*tau^3)/Tf^2;
end

figure;
set(gcf,'Color',[1 1 1]);   % White figure background

% s_cubic
subplot(3,2,1);
plot(t, s_cubic, 'b-', 'LineWidth', 2);
set(gca,'Color',[1 1 1]);   % White axes background
xlabel('Time (s)');
ylabel('s(t)');
title('Cubic Time Scaling s(t)');
grid on;

% s_quintic
subplot(3,2,2);
plot(t, s_quintic, 'r-', 'LineWidth', 2);
set(gca,'Color',[1 1 1]);
xlabel('Time (s)');
ylabel('s(t)');
title('Quintic Time Scaling s(t)');
grid on;

% ds_cubic
subplot(3,2,3);
plot(t, ds_cubic, 'b-', 'LineWidth', 2);
set(gca,'Color',[1 1 1]);
xlabel('Time (s)');
ylabel('ds/dt');
title('Cubic Time Scaling ds/dt');
grid on;

% ds_quintic
subplot(3,2,4);
plot(t, ds_quintic, 'r-', 'LineWidth', 2);
set(gca,'Color',[1 1 1]);
xlabel('Time (s)');
ylabel('ds/dt');
title('Quintic Time Scaling ds/dt');
grid on;

% dds_cubic
subplot(3,2,5);
plot(t, dds_cubic, 'b-', 'LineWidth', 2);
set(gca,'Color',[1 1 1]);
xlabel('Time (s)');
ylabel('ds^2/dt');
title('Cubic Time Scaling ds^2/dt');
grid on;

% dds_quintic
subplot(3,2,6);
plot(t, dds_quintic, 'r-', 'LineWidth', 2);
set(gca,'Color',[1 1 1]);
xlabel('Time (s)');
ylabel('ds^2/dt');
title('Quintic Time Scaling ds^2/dt');
grid on;
