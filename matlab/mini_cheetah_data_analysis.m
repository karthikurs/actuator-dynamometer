%% Load Data

logs = ["lcmlog-2021-02-21-2-powerfail",
    "lcmlog-2021-02-21-6-difficult-slippery",
    "lcmlog-2021-02-21-1-lcmfail",
    "lcmlog-2021-02-21-5-short",
    "lcmlog-2021-02-21-4-rightcircle",
    "lcmlog-2021-02-21-3-leftcircle"];

data_container = cell(length(logs),1);
for ii = 1:length(logs)
    logs(ii) = "mini-cheetah-logs/" + logs(ii) + ".mat";
    S = load(logs(ii));
    data_container{ii,1} = S;
end

%% Mini Cheetah Analysis

figure;
hold on;
m = 1;
n = 12;

leg_control_data = data_container{4,1}.leg_control_data;

for idx = m:n
    ii = idx - m + 1;
    vel = leg_control_data.qd(:,idx);
    tau = leg_control_data.tau_est(:,idx);
    vel_pts = linspace(-1,1,51); tau_pts = linspace(-8,8,51);
    N = histcounts2(vel,tau,vel_pts,tau_pts);

%     subplot(n-m+1,2,ii*2-1);
%     scat = scatter(vel,tau,'ko');
%     xlabel('velocity'); ylabel('torque');
%     scat.MarkerFaceAlpha = 0.05; scat.MarkerEdgeAlpha = 0.05;
%     set(gca, 'XLim', vel_pts([1 end]), 'YLim', tau_pts([1 end]), 'YDir', 'normal');

%     subplot(n-m+1,2,ii*2);
    subplot((n-m+1)/3,3,ii);
    imagesc(vel_pts, tau_pts, N',[0 50]);
    set(gca, 'XLim', vel_pts([1 end]), 'YLim', tau_pts([1 end]), 'YDir', 'normal');
    xlabel('velocity'); ylabel('torque');
end
sgtitle("MIT Mini Cheetah Actuator Data");
hold off

%% 

rms = [];
lengths = [];
trqs = [];
vels = [];
figure;
hold on;
for ii = 1:6
    for jj = 1:12
        rms = [rms, sqrt(mean(data_container{ii,1}.leg_control_data.tau_est(:,jj).^2))/.6];
        subplot(6,12,(ii-1)*12+(jj));
        plot(data_container{ii,1}.leg_control_data.tau_est(:,jj));
        
        trqs = [trqs; data_container{ii,1}.leg_control_data.tau_est(:,jj)];
        vels = [vels; data_container{ii,1}.leg_control_data.qd(:,jj)];
    end
    lengths = [lengths, data_container{ii,1}.leg_control_data.lcm_timestamp(end)];
end

% trqs = trqs./6;
% vels = vels.*6;

%%
M = [trqs, vels];
% writematrix(M, "mini_cheetah_torques_vels.csv");
dlmwrite("mini-cheetah-logs/mini_cheetah_torques_vels.csv", M, 'delimiter', ',', 'precision', '%.3f');

%%

vel = vels;
tau = trqs;
vel_pts = linspace(-1,1,51); tau_pts = linspace(-8,8,51);
N = histcounts2(vel,tau,vel_pts,tau_pts);

figure;
imagesc(vel_pts, tau_pts, N',[0 2000]);
set(gca, 'XLim', vel_pts([1 end]), 'YLim', tau_pts([1 end]), 'YDir', 'normal');
xlabel('velocity'); ylabel('torque');