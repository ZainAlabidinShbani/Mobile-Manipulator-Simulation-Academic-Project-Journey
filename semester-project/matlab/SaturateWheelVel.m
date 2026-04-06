function v = SaturateWheelVel(vel, max_val)
    v = min(max(vel, -max_val), max_val);
end
