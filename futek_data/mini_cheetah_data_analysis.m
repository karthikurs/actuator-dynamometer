%% Mini Cheetah Analysis

figure;
hold on;
m = 1;
n = 12;

for idx = m:n
    ii = idx - m + 1;
    vel = leg_control_data.v(:,idx);
    tau = leg_control_data.tau_est(:,idx);
    vel_pts = linspace(-2.5,2.5,51); tau_pts = linspace(-8,8,51);
    N = histcounts2(vel,tau,vel_pts,tau_pts);

%     subplot(n-m+1,2,ii*2-1);
%     scat = scatter(vel,tau,'ko');
%     xlabel('velocity'); ylabel('torque');
%     scat.MarkerFaceAlpha = 0.05; scat.MarkerEdgeAlpha = 0.05;
%     set(gca, 'XLim', vel_pts([1 end]), 'YLim', tau_pts([1 end]), 'YDir', 'normal');

%     subplot(n-m+1,2,ii*2);
    subplot((n-m+1)/3,3,ii);
    imagesc(vel_pts, tau_pts, N',[0 700]);
    set(gca, 'XLim', vel_pts([1 end]), 'YLim', tau_pts([1 end]), 'YDir', 'normal');
    xlabel('velocity'); ylabel('torque');
end
sgtitle("MIT Mini Cheetah Actuator Data");
hold off