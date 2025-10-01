function ExportTrajectory(x_log, theta_trajectories, N)
    data = [x_log(:,1:3), theta_trajectories', x_log(:,4:7), zeros(N-1, 1)];
    csvwrite('youbotmotion.csv',data);
end
