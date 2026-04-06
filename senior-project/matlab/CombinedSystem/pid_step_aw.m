function [u_sat, pid] = pid_step_aw(r, y, pid, dt, isAngle, umax)
if isAngle, e = wrapToPi(r - y); else, e = r - y; end
de = (e - pid.prevE) / max(dt,1e-9);

% candidate integral update
I_new = pid.I + e*dt;
I_new = max(-pid.Imax, min(pid.Imax, I_new));

u = pid.Kp*e + pid.Ki*I_new + pid.Kd*de;

% saturate
u_sat = max(-umax, min(umax, u));

% conditional integration: only accept I_new if not saturated (or helps desaturate)
if abs(u) <= umax || sign(u) ~= sign(u_sat)
    pid.I = I_new;
end

pid.prevE = e;
end