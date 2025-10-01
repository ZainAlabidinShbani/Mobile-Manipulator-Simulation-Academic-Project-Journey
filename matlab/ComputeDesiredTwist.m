function Vd = ComputeDesiredTwist(RefTraj, dt, N)
    Vd = zeros(6,1,N-1);
    for i = 1:N-1
        Trel = TransInv(RefTraj(:,:,i)) * RefTraj(:,:,i+1);
        Vd(:,:,i) = (1/dt) * se3ToVec(MatrixLog6(Trel));
    end
end
