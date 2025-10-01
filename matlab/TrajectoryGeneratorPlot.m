function [RefTraj, gripperstate, isCartesianSegment] = TrajectoryGeneratorPlot(Tse_initial,...
    Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)

    t_segment = [3,2,2,2,3,2,2,2]*k/0.01;
    total_len = sum(t_segment);

    RefTraj = zeros(4,4,total_len);
    gripperstate = zeros(1,total_len);
    isCartesianSegment = false(1,total_len); % Logical mask

    % 1. initial to standoff (Cartesian)
    T_se1 = CartesianTrajectory(Tse_initial, ...
        Tsc_initial*Tce_standoff, t_segment(1)*0.01/k,t_segment(1),5); 
    RefTraj(:,:,1:t_segment(1)) = reshape(cell2mat(T_se1),[4,4,t_segment(1)]);
    isCartesianSegment(1:t_segment(1)) = true;
    T_total = t_segment(1);

    % 2. standoff to grasping (Screw)
    T_se2 = ScrewTrajectory(Tsc_initial*Tce_standoff, ...
        Tsc_initial*Tce_grasp, t_segment(2)*0.01/k,t_segment(2),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(2)) = reshape(cell2mat(T_se2), [4,4,t_segment(2)]);
    % isCartesianSegment already false by default
    T_total = T_total + t_segment(2);

    % 3. holding position (Cartesian)
    T_se3 = CartesianTrajectory(Tsc_initial*Tce_grasp, ...
        Tsc_initial*Tce_grasp, t_segment(3)*0.01/k,t_segment(3),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(3)) = reshape(cell2mat(T_se3), [4,4,t_segment(3)]);
    gripperstate(1,T_total+1:T_total+t_segment(3)) = 1;
    isCartesianSegment(T_total+1:T_total+t_segment(3)) = true;
    T_total = T_total + t_segment(3);

    % 4. grasp to standoff (Screw)
    T_se4 = ScrewTrajectory(Tsc_initial*Tce_grasp, ...
        Tsc_initial*Tce_standoff, t_segment(4)*0.01/k,t_segment(4),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(4)) = reshape(cell2mat(T_se4), [4,4,t_segment(4)]);
    gripperstate(1,T_total+1:T_total+t_segment(4)) = 1;
    T_total = T_total + t_segment(4);

    % 5. standoff to standoff above final (Cartesian)
    T_se5 = CartesianTrajectory(Tsc_initial*Tce_standoff, ...
        Tsc_final*Tce_standoff, t_segment(5)*0.01/k,t_segment(5),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(5)) = reshape(cell2mat(T_se5), [4,4,t_segment(5)]);
    gripperstate(1,T_total+1:T_total+t_segment(5)) = 1;
    isCartesianSegment(T_total+1:T_total+t_segment(5)) = true;
    T_total = T_total + t_segment(5);

    % 6. standoff to final grasp pose (Screw)
    T_se6 = ScrewTrajectory(Tsc_final*Tce_standoff, ...
        Tsc_final*Tce_grasp, t_segment(6)*0.01/k,t_segment(6),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(6)) = reshape(cell2mat(T_se6), [4,4,t_segment(6)]);
    gripperstate(1,T_total+1:T_total+t_segment(6)) = 1;
    T_total = T_total + t_segment(6);

    % 7. holding at grasp (Cartesian)
    T_se7 = CartesianTrajectory(Tsc_final*Tce_grasp, ...
        Tsc_final*Tce_grasp, t_segment(7)*0.01/k,t_segment(7),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(7)) = reshape(cell2mat(T_se7), [4,4,t_segment(7)]);
    gripperstate(1,T_total+1:T_total+t_segment(7)) = 0;
    isCartesianSegment(T_total+1:T_total+t_segment(7)) = true;
    T_total = T_total + t_segment(7);

    % 8. releasing to standoff (Screw)
    T_se8 = ScrewTrajectory(Tsc_final*Tce_grasp, ...
        Tsc_final*Tce_standoff, t_segment(8)*0.01/k,t_segment(8),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(8)) = reshape(cell2mat(T_se8), [4,4,t_segment(8)]);
    gripperstate(1,T_total+1:T_total+t_segment(8)) = 0;
end
