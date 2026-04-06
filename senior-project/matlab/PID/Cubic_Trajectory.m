clear; clc; close all;

%% ====== Choose example segment values (change as you like) ======
p0 = 0;          % start position
pf = 1;          % final position
T  = 2;          % segment duration [s]

dt = 0.001;
t  = 0:dt:T;

%% ====== Cubic coefficients from boundary conditions ======
a0 = p0;
a1 = 0;
a2 = 3*(pf - p0)/T^2;
a3 = -2*(pf - p0)/T^3;

%% ====== Profiles ======
pd  = a0 + a1*t + a2*t.^2 + a3*t.^3;
dpd = a1 + 2*a2*t + 3*a3*t.^2;
ddpd= 2*a2 + 6*a3*t;

%% ====== Plots ======
figure('Color','w');

subplot(3,1,1);
plot(t,pd,'LineWidth',1.5); grid on;
ylabel('p_d(t)');
title('Cubic Segment Profiles (one axis)');

subplot(3,1,2);
plot(t,dpd,'LineWidth',1.5); grid on;
ylabel('p?_d(t)');

subplot(3,1,3);
plot(t,ddpd,'LineWidth',1.5); grid on;
ylabel('p?_d(t)');
xlabel('Time (s)');

% Mark boundary points (optional)
subplot(3,1,1); hold on;
plot(0,p0,'ko','MarkerFaceColor','k');
plot(T,pf,'ko','MarkerFaceColor','k');

subplot(3,1,2); hold on;
plot([0 T],[0 0],'ko','MarkerFaceColor','k');
