%% Pole location and root locus analysis (Task-space error dynamics)
clear; clc;

% --- Global graphics settings (force white background)
set(0,'DefaultFigureColor','w');
set(0,'DefaultAxesColor','w');
set(0,'DefaultAxesXColor','k');
set(0,'DefaultAxesYColor','k');
set(0,'DefaultAxesLineWidth',1.1);
set(0,'DefaultTextColor','k');

% --- Controller parameters (same philosophy as simulation)
zeta = 0.9;
wn   = 6;          % representative natural frequency

Kp = wn^2;
Kd = 2*zeta*wn;
Ki = 0.08*wn^3;

s = tf('s');

%% ===================== PD CONTROL (SECOND-ORDER) =====================
G = 1/s^2;                 % Error dynamics plant
C_pd = Kd*s + Kp;          % PD controller

T_pd = feedback(C_pd*G,1);

% --- Pole-zero map (PD)
figure('Color','w');
axes('Color','w'); hold on;
pzmap(T_pd);
grid on;
title('Closed-Loop Pole Locations (Task-Space PD Control)');
xlabel('Real Axis'); ylabel('Imaginary Axis');

%% ===================== ROOT LOCUS (GAIN VARIATION) =====================
figure('Color','w');
axes('Color','w'); hold on;
rlocus(C_pd*G);
grid on;
title('Root Locus of Task-Space Error Dynamics (PD Control)');
xlabel('Real Axis'); ylabel('Imaginary Axis');

%% ===================== PID CONTROL (THIRD-ORDER) =====================
C_pid = Kd*s + Kp + Ki/s;
T_pid = feedback(C_pid*G,1);

% --- Pole-zero map (PID)
figure('Color','w');
axes('Color','w'); hold on;
pzmap(T_pid);
grid on;
title('Closed-Loop Pole Locations with Integral Action (PID)');
xlabel('Real Axis'); ylabel('Imaginary Axis');
