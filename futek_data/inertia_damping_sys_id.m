%% Gaussian Random Process
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');

ri50_noback = "futek_test_05_05_2021_13-52-50.csv";
% ri50_back = "futek_test_05_05_2021_13-58-38.csv";
% ri50_back = "futek_test_06_05_2021_14-41-24.csv";
ri50_back = "futek_test_06_05_2021_17-07-59.csv";
ri50_back_val = "futek_test_06_05_2021_17-09-17.csv";

% no_act = "futek_test_05_05_2021_14-14-46.csv";
% no_act = "futek_test_06_05_2021_14-18-34.csv";
no_act = "futek_test_06_05_2021_17-01-09.csv";
no_act_val = "futek_test_06_05_2021_17-03-23.csv";

% no_act = "futek_test_06_05_2021_17-03-23.csv";
% no_act = "futek_test_10_05_2021_17-06-39.csv";
no_steel = "futek_test_06_05_2021_14-21-05.csv";
% ex8 = "futek_test_05_05_2021_17-35-01.csv";
% ex8 = "futek_test_06_05_2021_14-07-32.csv";
ex8_val = "futek_test_06_05_2021_17-21-28.csv";
ex8 = "futek_test_06_05_2021_17-22-41.csv";

datafiles = [no_act, ri50_back, ex8];
datafiles_val = [no_act_val, ri50_back_val, ex8_val];
titles = ["No Actuator", "RI50", "EX8", "No Steel"];
tfs = [];
Js = [];
Bs = [];
v_vafs=[];
q_vafs=[];
qd_vafs=[];
figure;
hold on;
for ii = 1:length(datafiles)
    datafile = datafiles(ii);    
    [Ts_res, v_exp, q_exp, t_exp] = load_grp_experiment(datafile, 0.0);
    
    vtf = tfest(v_exp,1,0,0);
    qtf = tfest(q_exp,1,0, NaN);
    qtfd = tfest(q_exp,1,0, NaN, 'Ts', Ts_res);
%     if ii==4
%          vtf = tfest(v_exp, 2, 1, 0, opts);
%          qtf = tfest(q_exp, 2, 1, 0, opts);
%     end
    ttf = tfest(t_exp,1,0,0);
%     if ii==1 || ii==4
%          vtf = tfest(v_exp, 2, 1, 0, opts);
%          qtf = tfest(q_exp, 2, 1, 0, opts);
%     end
    tfs = [tfs, qtf];
    Js = [Js, qtf.Denominator(1)/qtf.Numerator(1)];
    Bs = [Bs, qtf.Denominator(2)/qtf.Numerator(1)];
    
    datafile = datafiles_val(ii);    
    [Ts_res, v_exp, q_exp, t_exp] = load_grp_experiment(datafile, Ts_res);
    
    vsp = spa(v_exp);
    qsp = spa(q_exp);
    tsp = spa(t_exp);
    
    [Yvfit, vfit, ~] = compare(v_exp, vtf);
    [Yqfit, qfit, ~] = compare(q_exp, qtf);
    [Yqdfit, qdfit, ~] = compare(q_exp, qtfd);
    v_vafs = [v_vafs, vaf(v_exp.OutputData, Yvfit.OutputData)];
    q_vafs = [q_vafs, vaf(q_exp.OutputData, Yqfit.OutputData)];
    qd_vafs = [qd_vafs, vaf(q_exp.OutputData, Yqdfit.OutputData)];
    subplot(1,length(datafiles),ii)
    hold on
    h1 = bodeplot(etfe(q_exp), 'k');
    setoptions(h1,'FreqUnits','Hz','PhaseVisible','off');
    h2 = bodeplot(qtf,qtfd);
    setoptions(h2,'FreqUnits','Hz','PhaseVisible','off');
%     bode(qsp)
%     bodeplot(qtf)
%     bodeplot(c2d(qtf, Ts_res, 'zoh'))
%     bodeplot(qtfd)
    if ii==1 && false
        sys2 = tf([8000],[1 18]);
        bode(sys2)
    end
    title(sprintf("%s, Cont. VAF: %.2f%%, Disc. VAF: %.2f%%", titles(ii), q_vafs(ii), qd_vafs(ii)));
    legend(["etfe","tfest, c","tfest, d"],'location','southwest');
    hold off
end

J_ri50 = Js(2)-Js(1)
J_ex8 = Js(3)-Js(1)
B_ri50 = Bs(2)-Bs(1)
B_ex8 = Bs(3)-Bs(1)

% sgtitle(sprintf("RI50 Inertia = %.3e kgm^2, Damping = %.3e Nms/rad; EX8 Inertia = %.3e kgm^2, Damping = %.3e Nms/rad",...
%     J_ri50, B_ri50, J_ex8, B_ex8));

hold off;

%% Direction Viscous Damping

datafile1 = "futek_test_27_04_2021_18-05-31.csv";

data_table = readtable(datafile1,'PreserveVariableNames',true);
headers = data_table.Properties.VariableNames;

time_idx = find(ismember(headers,'time [s]'));
a1_v_cmd_idx = find(ismember(headers,'a1 velocity cmd [Hz]'));
a1_v_meas_idx = find(ismember(headers,'a1 velocity [rad/s]'));
a2_v_cmd_idx = find(ismember(headers,'a2 velocity cmd [Hz]'));
a2_v_meas_idx = find(ismember(headers,'a2 velocity [rad/s]'));
% trd_idx = find(ismember(headers,'trd605 torque [Nm]'));
trd_idx = find(ismember(headers,'trs605-5 torque [Nm]'));

time = table2array(data_table(1:end, time_idx));
a1_v_cmd = table2array(data_table(1:end, a1_v_cmd_idx));
a1_v_meas = table2array(data_table(1:end, a1_v_meas_idx));
a2_v_cmd = table2array(data_table(1:end, a2_v_cmd_idx));
a2_v_meas = table2array(data_table(1:end, a2_v_meas_idx));
trd = table2array(data_table(1:end, trd_idx));

buffer = 30;

a1_driving_idx_mask = abs(a1_v_cmd) > 0.01;
accel = a1_v_cmd - circshift(a1_v_cmd, 1);
for jj = -buffer:buffer
    a1_driving_idx_mask = a1_driving_idx_mask & (abs(circshift(accel, jj)) < 0.1);
end

a2_driving_idx_mask = abs(a2_v_cmd) > 0.01;
accel = a2_v_cmd - circshift(a2_v_cmd, 1);
for jj = -buffer:buffer
    a2_driving_idx_mask = a2_driving_idx_mask & (abs(circshift(accel, jj)) < 0.1);
end

figure;
hold on;
scatter(a1_v_meas(a2_driving_idx_mask), -trd(a2_driving_idx_mask), 'b+', 'DisplayName','Friction Torque in A1 while A2 Drives')
scatter(a2_v_meas(a1_driving_idx_mask), -trd(a1_driving_idx_mask), 'ro', 'DisplayName','Friction Torque in A2 while A1 Drives')
title('Friction Torque over Velocity')
legend('Location','Best')
xlabel('Rotational Velocity $(\omega)$ [rad/s]');
ylabel('Torque $(\tau)$ [Nm]');
hold off;

function [Ts_res, v_exp, q_exp, t_exp] = load_grp_experiment(datafile, Ts_res)
    data_table = readtable(datafile,'PreserveVariableNames',true);
    headers = data_table.Properties.VariableNames;

    time_idx = find(ismember(headers,'time [s]'));
    a1_v_idx = find(ismember(headers,'a1 velocity [rad/s]'));
    a1_q_idx = find(ismember(headers,'a1 q-axis [A]'));
    ts_idx = find(ismember(headers,'trs605-5 torque [Nm]'));

    time = table2array(data_table(1:end, time_idx));
    a1_v = table2array(data_table(1:end, a1_v_idx));
    a1_q = table2array(data_table(1:end, a1_q_idx))*0.105;
    ts = table2array(data_table(1:end, ts_idx));

    dt = abs(time - circshift(time, 1));
    Ts = median(dt)
    
    ratio = 0.9;
    
    if Ts_res == 0
        Ts_res = Ts/ratio;
    end
    
    a1_v = resample(a1_v, time, 1/Ts_res, 'pchip');
    a1_q = resample(a1_q, time, 1/Ts_res, 'pchip');
    ts = resample(ts, time, 1/Ts_res, 'pchip');
%     ts = lowpass(ts, 21, 1/Ts, 'Steepness', 0.99);
%     a1_v = lowpass(a1_v, 21, 1/Ts, 'Steepness', 0.99);
    v_exp = iddata(a1_v, ts, Ts_res);
    q_exp = iddata(a1_v, a1_q, Ts_res);
    t_exp = iddata(ts, a1_q, Ts_res);
end