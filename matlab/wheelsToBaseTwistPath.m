function V_des_all = wheelsToBaseTwistPath(w_des_all)
    % wheelsToBaseTwistPath: Convert desired wheel velocities over time to base twists
    %
    % Inputs:
    %   w_des_all: 4xN matrix of wheel velocities over time
    %
    % Outputs:
    %   V_des_all: 3xN matrix of base twists [phidot; xdot; ydot] over time
    
    % Robot parameters
    r = 0.0475;      % wheel radius (meters)
    l = 0.29;        % half length of the base (meters)
    w = 0.188;       % half width of the base (meters)
    lw = l + w;      % sum for Jacobian
    
    % Forward kinematic matrix (wheel velocities ? base twist)
    H_p = (r / 4) * [ 
        -1/(lw),  1/(lw),  1/(lw), -1/(lw);
         1,       1,       1,       1;
        -1,       1,      -1,       1];
    
    % Number of time steps
    N = size(w_des_all, 2);
    
    % Preallocate output
    V_des_all = zeros(3, N);
    
    % Convert wheel velocities to twists at each time step
    for k = 1:N
        V_des_all(:, k) = H_p * w_des_all(:, k);
    end
end
