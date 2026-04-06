function [pd,dpd,ddpd,seg_id] = pickplace_cubic(t,Ts,p_start,p_pick,p_place,z_lift)
T1=Ts(1); T2=Ts(2); T3=Ts(3); T4=Ts(4); T5=Ts(5);
p_lift1 = [p_pick(1);  p_pick(2);  z_lift];
p_lift2 = [p_place(1); p_place(2); z_lift];
if t <= T1
    seg_id=1; [pd,dpd,ddpd]=cubic_segment(t,T1,p_start,p_pick);
elseif t <= T1+T2
    seg_id=2; [pd,dpd,ddpd]=cubic_segment(t-T1,T2,p_pick,p_lift1);
elseif t <= T1+T2+T3
    seg_id=3; [pd,dpd,ddpd]=cubic_segment(t-(T1+T2),T3,p_lift1,p_lift2);
elseif t <= T1+T2+T3+T4
    seg_id=4; [pd,dpd,ddpd]=cubic_segment(t-(T1+T2+T3),T4,p_lift2,p_place);
else
    seg_id=5; [pd,dpd,ddpd]=cubic_segment(t-(T1+T2+T3+T4),T5,p_place,p_start);
end
end