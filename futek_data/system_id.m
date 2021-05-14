%% motor temp to PLA temp model estimation
% e1 = load_temp_experiment('futek_test_13_04_2021_20-10-36.csv');
swap_temps=false;
[e1, z1] = load_experiments('futek_test_13_04_2021_20-10-36.csv', swap_temps);

NN2 = struc(1:3,1:3,1000);
% selstruc(arxstruc(e1(:,:,1),e1(:,:,1),NN2))

etf = tfest(e1,2,0,0)

% e2 = load_temp_experiment('futek_test_14_04_2021_16-17-17.csv');
% e3 = load_temp_experiment('futek_test_14_04_2021_16-13-41.csv');
[e2, z2] = load_experiments('futek_test_14_04_2021_16-17-17.csv', swap_temps);
[e3, z3] = load_experiments('futek_test_14_04_2021_16-13-41.csv', swap_temps);

compare(e3, etf)

%% current cmd to motor temp estimation
% z1 = load_current_experiment('futek_test_13_04_2021_20-10-36.csv');
% NN2 = struc(1:3,1:3,10);
% selstruc(arxstruc(e1(:,:,1),e1(:,:,1),NN2))
% z1 = z1(1:5000);

tfopt = tfestOptions('InitialCondition','estimate');
ztf = tfest(z1,2,1,0)

% z2 = load_current_experiment('futek_test_14_04_2021_16-17-17.csv');
% z3 = load_current_experiment('futek_test_14_04_2021_16-13-41.csv');

compare(z1, ztf)

%%

set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');

datafiles = ["futek_test_17_04_2021_17-38-27.csv",
    "futek_test_17_04_2021_17-38-55.csv",
    "futek_test_17_04_2021_17-39-55.csv",
    "futek_test_17_04_2021_17-41-49.csv"];

datafiles = ["futek_test_21_04_2021_15-48-50.csv"];
% datafiles = ["futek_test_21_04_2021_14-43-21.csv"];
g = 6;

v_a1s = [];
t_a1s = [];
t_trds = [];
t_ests = [];
ps = [];
figure;

for ii = 1:length(datafiles)

    datafile1 = datafiles(ii);
    data_table = readtable(datafile1);
    t = table2array(data_table(1:end, 1));
    
    fs = 1/mean(-t(1:end-1) + t(2:end));
    fpass = 0.1*fs;
    
    i_cmd = table2array(data_table(1:end, 2));
    di_cmd = i_cmd(2:end) - i_cmd(1:end-1); di_cmd = [di_cmd; di_cmd(end)];
    t_cmd = table2array(data_table(1:end, 3));
    v_a1 = table2array(data_table(1:end, 5));
    t_a1 = table2array(data_table(1:end, 6));
    t_a2 = table2array(data_table(1:end, 9));
    t_trd = table2array(data_table(1:end, 24));
    kt = table2array(data_table(1:end, 27));
    
    v_a1_filt = lowpass(v_a1, fpass, fs);
    t_a1_filt = lowpass(t_a1, fpass, fs);
    t_a2_filt = lowpass(t_a2, fpass, fs);
    t_trd_filt = lowpass(t_trd, fpass, fs);
    kt_filt = lowpass(kt, fpass, fs);
    
    idx_array = (i_cmd ~= 0).*(t > 1).*(t < 10);
    for jj = -3:3
        idx_array = idx_array.*(circshift(di_cmd, jj) == 0);
    end
    idx_array = idx_array==1;
    kt_a1_mean = mean(t_a1_filt(idx_array)./i_cmd(idx_array))/g
    kt_trd_mean = mean(t_trd_filt(idx_array)./i_cmd(idx_array))/g
    kt_a1_std = std(t_a1_filt(idx_array)./i_cmd(idx_array))/g
    kt_trd_std = std(t_trd_filt(idx_array)./i_cmd(idx_array))/g

    [p, s] = polyfit(v_a1, t_trd, 1);
    t_est = polyval(p, v_a1);
    
    v_a1s = [v_a1s; v_a1];
    t_a1s = [t_a1s; t_a1];
    t_trds = [t_trds; t_trd];
    t_ests = [t_ests; t_est];
    ps = [ps; p];
% 	scatter(v_a1, t_trd,'.');
% 	plot(v_a1, t_est, '-', 'DisplayName',"data idx: "+ii+"; damping ="+p(1)+" Nm/rad/s; coloumbic ="+p(2)+" Nm",'LineWidth',4);
% 	scatter(v_a1, kt, 'DisplayName','$K_T$');
% 	plot(t, kt)
    subplot(3,1,1)
    hold on;
    plot(t, t_cmd,'DisplayName','Command');
    plot(t, t_a1_filt,'DisplayName','Actuator 1');
    plot(t, t_a2_filt,'DisplayName','Actuator 2');
    plot(t, t_trd_filt,'DisplayName','TRD605');
%     ylim([0,4])
    xlabel("time [s]");
    ylabel("torque [Nm]");
    legend('location','best');
    hold off;
    
    subplot(3,1,2)
    hold on;
    plot(t, t_a1_filt - t_trd_filt);
    plot(t, t_trd_filt - t_a2_filt);
    xlabel("time [s]");
    ylabel("torque [Nm]");
    hold off;
    
    subplot(3,1,3)
    hold on;
    plot(t, t_trd_filt./i_cmd/g);
    plot(t, t_a1_filt./i_cmd/g);
    xlabel("time [s]");
    ylabel("torque constant [Nm/A]");
    hold off;
    
    figure;
    hold on;
    plot(t, t_cmd,'DisplayName','Command');
    plot(t, t_a1_filt,'DisplayName','Actuator 1');
    plot(t, t_a2_filt,'DisplayName','Actuator 2');
    plot(t, t_trd_filt,'DisplayName','TRD605');
%     ylim([0,4])
    xlabel("time [s]");
    ylabel("torque [Nm]");
    legend('location','best');
    hold off;
end
% legend('location','best');
ps
% xlabel("velocity [rad/s]")
% ylabel("TRD506 torque [mNm]")
% xlabel("time [s]");
% ylabel("torque [Nm]");
% title("Damping Coeff ~"+p(1)+" Nm/rad/s; Coloumbic Friction ~"+p(2)+" Nm");

%% Thermal Plotting
% [e4, z4] = load_experiments('futek_test_17_04_2021_16-37-13.csv', true);
% [e5, z5] = load_experiments('futek_test_17_04_2021_17-20-44.csv', true);

% plot_thermal_model('futek_test_17_04_2021_16-37-13.csv', etf, ztf, true);

plot_thermal_model('futek_test_27_04_2021_15-46-08.csv', etf, ztf, true, true);

%% Step response plotting

datafile = "futek_test_28_04_2021_21-30-24.csv";
datafile = "futek_test_28_04_2021_21-28-31.csv";
datafile = "futek_test_28_04_2021_22-11-23.csv";

data_table = readtable(datafile1,'PreserveVariableNames',true);
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
ts = lowpass(ts, 1, 1/Ts);
a1_v = lowpass(a1_v, 1, 1/Ts);

% v_exp = iddata(a1_v, a1_q, Ts);
% vtf = tfest(v_exp,1,0,0);
% compare(v_exp, vtf);

damping_exp = iddata(a1_v, ts, Ts);
tfopt = tfestOptions('InitialCondition','zero');
dtf = tfest(damping_exp, 1, 0, 0, tfopt)
figure;
compare(damping_exp, dtf);

%% Gaussian Random Process
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');

ri50_noback = "futek_test_05_05_2021_13-52-50.csv";
% ri50_back = "futek_test_05_05_2021_13-58-38.csv";
% ri50_back = "futek_test_06_05_2021_14-41-24.csv";
ri50_back = "futek_test_06_05_2021_17-09-17.csv";
% no_act = "futek_test_05_05_2021_14-14-46.csv";
% no_act = "futek_test_06_05_2021_14-18-34.csv";
no_act = "futek_test_06_05_2021_17-03-23.csv";
no_steel = "futek_test_06_05_2021_14-21-05.csv";
% ex8 = "futek_test_05_05_2021_17-35-01.csv";
% ex8 = "futek_test_06_05_2021_14-07-32.csv";
ex8 = "futek_test_06_05_2021_17-22-41.csv";

datafiles = [no_act, ri50_back, ex8, no_steel];
titles = ["No Actuator", "RI50", "EX8", "No Steel"];
tfs = [];
Js = [];
Bs = [];
v_vafs=[];
q_vafs=[];
figure;
hold on;
for ii = 1:length(datafiles)
    datafile = datafiles(ii);
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
    
    a1_v = resample(a1_v, time, ratio/Ts, 'pchip');
    a1_q = resample(a1_q, time, ratio/Ts, 'pchip');
    ts = resample(ts, time, ratio/Ts, 'pchip');
%     ts = lowpass(ts, 21, 1/Ts, 'Steepness', 0.99);
%     a1_v = lowpass(a1_v, 21, 1/Ts, 'Steepness', 0.99);

    opts = tfestOptions('WeightingFilter',[5.28, 668]);
    v_exp = iddata(a1_v, ts, Ts/ratio);
    q_exp = iddata(a1_v, a1_q, Ts/ratio);
    vtf = tfest(v_exp,1,0,0);
    qtf = tfest(q_exp,1,0,0);
    if ii==4
         vtf = tfest(v_exp, 2, 1, 0, opts);
         qtf = tfest(q_exp, 2, 1, 0, opts);
    end
    tfs = [tfs, vtf];
    Js = [Js, qtf.Denominator(1)/qtf.Numerator(1)];
    Bs = [Bs, qtf.Denominator(2)/qtf.Numerator(1)];
    vsp = spa(v_exp);
    qsp = spa(q_exp);
%     figure;
    if ii==4
        break
    end
    [Yvfit, vfit, ic] = compare(v_exp, vtf);
    [Yqfit, qfit, ic] = compare(q_exp, qtf);
    v_vafs = [v_vafs, vaf(v_exp.OutputData, Yvfit.OutputData)];
    q_vafs = [q_vafs, vaf(q_exp.OutputData, Yqfit.OutputData)];
    subplot(1,length(datafiles),ii)
    hold on
    bode(qsp)
    bode(qtf)
    title(sprintf("%s, VAF: %.2f%%", titles(ii), q_vafs(ii)));
    legend(["moteus data","moteus est"],'location','southwest');
    hold off
end

J_ri50 = Js(2)-Js(1)
J_ex8 = Js(3)-Js(1)
B_ri50 = Bs(2)-Bs(1)
B_ex8 = Bs(3)-Bs(1)

% sgtitle(sprintf("RI50 Inertia = %.3e kgm^2, Damping = %.3e Nms/rad; EX8 Inertia = %.3e kgm^2, Damping = %.3e Nms/rad",...
%     J_ri50, B_ri50, J_ex8, B_ex8));

hold off;
%% KT

% data_no_gb_stall_fnames_a1 = ["futek_test_23_04_2021_16-38-38.csv",
%     "futek_test_23_04_2021_17-15-16.csv",
%     "futek_test_23_04_2021_17-18-16.csv",
%     "futek_test_23_04_2021_18-06-56.csv",
%     "futek_test_23_04_2021_18-10-46.csv",
%     "futek_test_23_04_2021_18-37-08.csv",
%     "futek_test_23_04_2021_20-03-31.csv"];
data_no_gb_stall_fnames_a1 = ["futek_test_27_04_2021_16-06-17.csv",
    "futek_test_27_04_2021_16-59-45.csv"];
% data_no_gb_damp_fnames_a1 = ["futek_test_23_04_2021_19-52-25.csv",
%     "futek_test_23_04_2021_19-57-18.csv"];
data_no_gb_damp_fnames_a1 = ["futek_test_27_04_2021_15-46-08.csv",
    "futek_test_27_04_2021_16-28-22.csv"];

data_no_gb_stall_fnames_a2 = ["futek_test_23_04_2021_18-52-31.csv",
    "futek_test_23_04_2021_19-08-29.csv",
    "futek_test_23_04_2021_19-29-38.csv",
    "futek_test_23_04_2021_19-30-41.csv",
    "futek_test_23_04_2021_19-36-34.csv"];
data_no_gb_damp_fnames_a2 = ["futek_test_23_04_2021_19-16-12.csv",
    "futek_test_23_04_2021_19-20-04.csv",
    "futek_test_23_04_2021_19-22-22.csv"];

datafiles = ["futek_test_23_04_2021_16-38-38.csv"];
datafiles = data_no_gb_stall_fnames_a1;

condense = false;

alpha = 0.75;

figure;
for kk = 1:4
    label = '';
    if kk==1; datafiles = data_no_gb_stall_fnames_a1;   label = 'S, O1, ';      end
    if kk==2; datafiles = data_no_gb_damp_fnames_a1;    label = 'D, O1, ';      end
    if kk==3; datafiles = data_no_gb_stall_fnames_a2;   label = 'S, O2, '; continue;     end
    if kk==4; datafiles = data_no_gb_damp_fnames_a2;    label = 'D, O2, '; continue;     end
    time_oall = [];
    a1_q_cmd_oall = [];
    a1_q_meas_oall = [];
    a2_q_cmd_oall = [];
    a2_q_meas_oall = [];
    trd_oall = [];
    num_files = length(datafiles);
    for ii = 1:num_files
        color = (ii)/(num_files);
        color = 0.25+0.5*color;
        color_r = [color, 0, 0];
        color_g = [0, color, 0];
        color_b = [0, 0, color];
        color_grey = [color, color, color];

        datafile1 = datafiles(ii);
        data_table = readtable(datafile1,'PreserveVariableNames',true);
        headers = data_table.Properties.VariableNames;

        time_idx = find(ismember(headers,'time [s]'));
        a1_q_cmd_idx = find(ismember(headers,'a1 q-axis cmd [A]'));
        a1_q_meas_idx = find(ismember(headers,'a1 q-axis [A]'));
        a2_q_cmd_idx = find(ismember(headers,'a2 q-axis cmd [A]'));
        a2_q_meas_idx = find(ismember(headers,'a2 q-axis [A]'));
%         trd_idx = find(ismember(headers,'trd605 torque [Nm]'));
        trd_idx = find(ismember(headers,'trs605-5 torque [Nm]'));

        a1_v_idx = find(ismember(headers,'a1 velocity [rad/s]'));

        time = table2array(data_table(1:end, time_idx));
        a1_q_cmd = table2array(data_table(1:end, a1_q_cmd_idx));
        a1_q_meas = table2array(data_table(1:end, a1_q_meas_idx));
        a2_q_cmd = table2array(data_table(1:end, a2_q_cmd_idx));
        a2_q_meas = table2array(data_table(1:end, a2_q_meas_idx));
        trd = table2array(data_table(1:end, trd_idx));

        a1_v = table2array(data_table(1:end, a1_v_idx));
        if mean(abs(a1_v)) > 1.0
            fprintf("non-zero velocity: %f\n", mean(abs(a1_v)));
            fprintf(datafile1);
            fprintf("\n");
        end

        fs = 1/mean(-time(1:end-1) + time(2:end));
        fpass = 0.1*fs;

        a1_driving_idx_mask = abs(a1_q_cmd) > 0.01;
        for jj = -10:10
            a1_driving_idx_mask = a1_driving_idx_mask & (abs(circshift(a1_q_cmd, jj)) > 0.01);
        end
        
        a2_driving_idx_mask = abs(a2_q_cmd) > 0.01;
        for jj = -10:10
            a2_driving_idx_mask = a2_driving_idx_mask & (abs(circshift(a2_q_cmd, jj)) > 0.01);
        end
        
        a1_driven_idx_mask = a2_driving_idx_mask;
        a2_driven_idx_mask = a1_driving_idx_mask;

        time_oall = [time_oall; time];
        a1_q_cmd_oall = [a1_q_cmd_oall; a1_q_cmd];
        a1_q_meas_oall = [a1_q_meas_oall; a1_q_meas];
        a2_q_cmd_oall = [a2_q_cmd_oall; a2_q_cmd];
        a2_q_meas_oall = [a2_q_meas_oall; a2_q_meas];
        trd_oall = [trd_oall; trd];
        
        x1 = a1_q_meas(a1_driving_idx_mask); y1 = trd(a1_driving_idx_mask);
        x2 = a1_q_meas(a1_driven_idx_mask); y2 = trd(a1_driven_idx_mask);
        x3 = a2_q_meas(a2_driving_idx_mask); y3 = trd(a2_driving_idx_mask);
        x4 = a2_q_meas(a2_driven_idx_mask); y4 = trd(a2_driven_idx_mask);
        
        if condense
            [x1,y1] = condense_data(x1, y1, a1_q_cmd(a1_driving_idx_mask));
            [x2,y2] = condense_data(x2, y2, a2_q_cmd(a2_driving_idx_mask));
            [x3,y3] = condense_data(x3, y3, a2_q_cmd(a2_driving_idx_mask));
            [x4,y4] = condense_data(x4, y4, a1_q_cmd(a1_driving_idx_mask));
        end
        
        subplot(2,2,1);
        hold on
        s = scatter(x1, y1,...
            '+','MarkerEdgeColor', color_grey, 'DisplayName',char(['\verb|',char(datafile1),'|, a1 driving']),...
            'HandleVisibility','off');
        s.MarkerFaceAlpha = alpha;
        s.MarkerEdgeAlpha = alpha;
        
        s = scatter(x2, y2,...
            'o','MarkerEdgeColor', color_r, 'DisplayName',char(['\verb|',char(datafile1),'|, a1 driven']),...
            'HandleVisibility','off');
        s.MarkerFaceAlpha = alpha;
        s.MarkerEdgeAlpha = alpha;
        
        ylabel('Torque [Nm]');
        xlabel('q-axis Current ($i_q$) [A]');
        hold off
        
        subplot(2,2,2)
        hold on
        s = scatter(x3, y3,...
            '+','MarkerEdgeColor', color_grey, 'DisplayName',char(['\verb|',char(datafile1),'|, a2 driving']),...
            'HandleVisibility','off');
        s.MarkerFaceAlpha = alpha;
        s.MarkerEdgeAlpha = alpha;
        
        s = scatter(x4, y4,...
            'o','MarkerEdgeColor', color_r, 'DisplayName',char(['\verb|',char(datafile1),'|, a2 driven']),...
            'HandleVisibility','off');
        s.MarkerFaceAlpha = alpha;
        s.MarkerEdgeAlpha = alpha;
        
        ylabel('Torque [Nm]');
        xlabel('q-axis Current ($i_q$) [A]');
        hold off
        
        subplot(2,2,3);
        hold on
        s = scatter(x1, y1,...
            '+','MarkerEdgeColor', color_grey, 'DisplayName',char(['\verb|',char(datafile1),'|, a1 driving']),...
            'HandleVisibility','off');
        s.MarkerFaceAlpha = alpha;
        s.MarkerEdgeAlpha = alpha;
        
        s = scatter(x2, y2,...
            'o','MarkerEdgeColor', color_r, 'DisplayName',char(['\verb|',char(datafile1),'|, a1 driven']),...
            'HandleVisibility','off');
        s.MarkerFaceAlpha = alpha;
        s.MarkerEdgeAlpha = alpha;
        
        ylabel('Torque [Nm]');
        xlabel('q-axis Current ($i_q$) [A]');
        hold off
        
        subplot(2,2,4)
        hold on
        s = scatter(x3, y3,...
            '+','MarkerEdgeColor', color_grey, 'DisplayName',char(['\verb|',char(datafile1),'|, a2 driving']),...
            'HandleVisibility','off');
        s.MarkerFaceAlpha = alpha;
        s.MarkerEdgeAlpha = alpha;
        
        s = scatter(x4, y4,...
            'o','MarkerEdgeColor', color_r, 'DisplayName',char(['\verb|',char(datafile1),'|, a2 driven']),...
            'HandleVisibility','off');
        s.MarkerFaceAlpha = alpha;
        s.MarkerEdgeAlpha = alpha;
        
        ylabel('Torque ($\tau$) [Nm]');
        xlabel('q-axis Current ($i_q$) [A]');
        hold off

    end
    a1_driving_idx_mask = abs(a1_q_cmd_oall) > 0.01;
    for jj = -10:10
        a1_driving_idx_mask = a1_driving_idx_mask & (abs(circshift(a1_q_cmd_oall, jj)) > 0.01);
    end
    
    a2_driving_idx_mask = abs(a2_q_cmd_oall) > 0.01;
    for jj = -10:10
        a2_driving_idx_mask = a2_driving_idx_mask & (abs(circshift(a2_q_cmd_oall, jj)) > 0.01);
    end
    
    a1_driven_idx_mask = a2_driving_idx_mask; a2_driven_idx_mask = a1_driving_idx_mask;
    
    %
    x_a1_driving = a1_q_meas_oall(a1_driving_idx_mask); y_a1_driving = trd_oall(a1_driving_idx_mask);
    [x_a1_driving,sortIdx] = sort(x_a1_driving,'ascend'); y_a1_driving = y_a1_driving(sortIdx);
    %
    x_a2_driving = a2_q_meas_oall(a2_driving_idx_mask); y_a2_driving = trd_oall(a2_driving_idx_mask);
    [x_a2_driving,sortIdx] = sort(x_a2_driving,'ascend'); y_a2_driving = y_a2_driving(sortIdx);
    %
    x_a1_driven = a1_q_meas_oall(a1_driven_idx_mask); y_a1_driven = trd_oall(a1_driven_idx_mask);
    [x_a1_driven,sortIdx] = sort(x_a1_driven,'ascend'); y_a1_driven = y_a1_driven(sortIdx);
    %
    x_a2_driven = a2_q_meas_oall(a2_driven_idx_mask); y_a2_driven = trd_oall(a2_driven_idx_mask);
    [x_a2_driven,sortIdx] = sort(x_a2_driven,'ascend'); y_a2_driven = y_a2_driven(sortIdx);
    %
    
    x1 = x_a1_driving; y1 = y_a1_driving;
    x2 = x_a2_driving; y2 = y_a2_driving;
    x3 = x_a1_driven; y3 = y_a1_driven;
    x4 = x_a2_driven; y4 = y_a2_driven;

    if condense
        [x1,y1] = condense_data(x1, y1, a1_q_cmd_oall(a1_driving_idx_mask));
        [x2,y2] = condense_data(x2, y2, a2_q_cmd_oall(a2_driving_idx_mask));
        [x3,y3] = condense_data(x3, y3, a2_q_cmd_oall(a2_driving_idx_mask));
        [x4,y4] = condense_data(x4, y4, a1_q_cmd_oall(a1_driving_idx_mask));
    end

    subplot(2,2,1); hold on
    
    x = x1; y = y1; order = 1;
    [p, x, y, est] = polynom_fit(x, y, order);
    plot(x, est,'k-','LineWidth',2,'DisplayName',sprintf("%s a1 Driving, $\\tau = %.3f i_q + %.3f$", label, p(1),p(2)));
    
    x = x3; y = y3;
    [p, x, y, est] = polynom_fit(x, y, order);
    plot(x, est,'r-','LineWidth',2,'DisplayName',sprintf("%s a1 Driven, $\\tau = %.3f i_q + %.3f$", label, p(1),p(2)));
    
    legend('location','best'); title('Actuator 1 Linear Fits')
    hold off
    %
    subplot(2,2,2); hold on
    
    x = x2; y = y2;
    [p, x, y, est] = polynom_fit(x, y, order);
    plot(x, est,'k-','LineWidth',2,'DisplayName',sprintf("%s a2 Driving, $\\tau = %.3f i_q + %.3f$", label, p(1),p(2)));
    
    x = x4; y = x4;
    [p, x, y, est] = polynom_fit(x, y, order);
    plot(x, est,'r-','LineWidth',2,'DisplayName',sprintf("%s a2 Driven, $\\tau = %.3f i_q + %.3f$", label, p(1),p(2)));
    
    legend('location','best'); title('Actuator 2 Linear Fits')
    hold off
    %
    subplot(2,2,3); hold on
    x = x1; y = y1; order = 3;
    [p, x, y, est] = polynom_fit(x, y, order);
    plot(x, est,'k:','LineWidth',2,'DisplayName',...
        sprintf("%s a1 Driving, $\\tau = %.3f i_q^3 + %.3f i_q^2 + %.3f i_q + %.3f$", label, p(1),p(2),p(3),p(4)));
    
    x = x3; y = y3;
    [p, x, y, est] = polynom_fit(x, y, order);
    plot(x, est,'r:','LineWidth',2,'DisplayName',...
        sprintf("%s a1 Driven, $\\tau = %.3f i_q^3 + %.3f i_q^2 + %.3f i_q + %.3f$", label, p(1),p(2),p(3),p(4)));
    
    legend('location','best'); title('Actuator 1 Cubic Fits')
    hold off
    %
    subplot(2,2,4); hold on
    
    x = x2; y = y2;
    [p, x, y, est] = polynom_fit(x, y, order);
    plot(x, est,'k:','LineWidth',2,'DisplayName',...
        sprintf("%s a2 Driving, $\\tau = %.3f i_q^3 + %.3f i_q^2 + %.3f i_q + %.3f$", label, p(1),p(2),p(3),p(4)));
    
    x = x4; y = y4;
    [p, x, y, est] = polynom_fit(x, y, order);
    plot(x, est,'r:','LineWidth',2,'DisplayName',...
        sprintf("%s a2 Driven, $\\tau = %.3f i_q^3 + %.3f i_q^2 + %.3f i_q + %.3f$", label, p(1),p(2),p(3),p(4)));    
    
    legend('location','best'); title('Actuator 2 Cubic Fits')
    hold off
end
sgtitle('Torque $\tau$ vs. q-axis Current $i_q$ for Stalled and Damped Load')

%% Viscous Damping

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

function [q_cond, t_cond] = condense_data(q_meas, t_meas, q_cmd)
    cmds = unique(q_cmd);
    q_cond = cmds;
    t_cond = cmds;
    
    for ii = 1:length(cmds)
        mask = q_cmd == cmds(ii);
        q_meas_local = q_meas(mask);
        t_meas_local = t_meas(mask);
        q_cond(ii) = mean(q_meas_local);
        t_cond(ii) = mean(t_meas_local);
    end
end

function v = vaf(y, y_est)

    % Variance Accounted For (VAF) | Percentage value (%)
    %
    % v = vaf(y, y_est)
    %
    % y     : measured output (real)
    % y_est : estimated output
    %
    
    v = var(y - y_est) / var(y) ;
    v = 100 * ( 1 - v );
    
    if ( v < 0 )
        v = 0;
    end

end


function [p, x, y, est] = polynom_fit(x, y, order)
    p = polyfit(x, y, order);
    est = polyval(p, x);
end

function plot_thermal_model(dataset, etf, ztf, swap_temps, alt)
    set(0, 'DefaultTextInterpreter', 'latex');
    set(0, 'DefaultLegendInterpreter', 'latex');
    set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
    % dataset = 'futek_test_17_04_2021_16-37-13.csv';

    if ~alt
        [e, z] = load_experiments(dataset, swap_temps);
    end
    [e, z] = load_experiments_alt(dataset);

    [yfit, ~, ~] = compare(z, ztf);
    e_hat = iddata(e.y, yfit.y, e.Ts);
    [yfit, ~, ~] = compare(e_hat, etf);
    c_hat = iddata(yfit.y, z.u, e.Ts);
    c_true = iddata(e.y, z.u, e.Ts);

    figure;
    
    set(gcf, 'Units', 'inches')
    set(gcf, 'Position', [0 0 6 7.5]);
    
    subplot(3,1,1)
    plot(z.SamplingInstants, z.u, 'b-', 'DisplayName','$i^2$ motor input');
    ylabel({"Square of input current";"$i_q^2$ [A$^2$]"})
    title('Motor Current Command')

    subplot(3,1,2)
    hold on
    plot(e.SamplingInstants, e.u, 'k-', 'DisplayName', 'Motor Temp Meas')
    plot(e_hat.SamplingInstants, e_hat.u, 'b--', 'DisplayName', 'Motor Temp Est')
    err = mean((e.u-e_hat.u).^2);
    title("Motor Temp Above Ambient (MSE = "+err+" $^o $C$^2$)")
    ylabel({"Motor Temperature";"$T_m$, [$^o$ C]"})
    ylim([0 70]);
    legend('location','best');
    hold off

    subplot(3,1,3)
    hold on
    plot(c_true.SamplingInstants, c_true.y, 'k-', 'DisplayName','PLA Temp Meas')
    plot(c_hat.SamplingInstants, c_hat.y, 'b--', 'DisplayName','PLA Temp Est')
    err = mean((c_true.y-c_hat.y).^2);
    title("PLA Temp Above Ambient (MSE = "+err+" $^o $C$^2$)")
    ylabel({"PLA Temperature";"$T_P$, [$^o$ C]"})
    ylim([0 70]);
    xlabel("Time, $t$, [s]")
    legend('location','best');
    hold off;

    sgtitle(dataset, 'interpreter','none');
    
    saveas(gcf,"thermal_model_"+dataset+".png")
end

function exp = load_temp_experiment(fname, swap_temp)
    data_table = readtable(fname);
    
    t = table2array(data_table(1:end, 1));
    i = table2array(data_table(1:end, 2));
    i2 = i.^2;

    if swap_temp
        Tp = table2array(data_table(1:end, 26)) - 23;
        Tm = table2array(data_table(1:end, 25)) - 23;
    else
        Tp = table2array(data_table(1:end, 25)) - 23;
        Tm = table2array(data_table(1:end, 26)) - 23;
    end
    
    Ts = mean(-t(1:end-1) + t(2:end));
    
    exp = iddata(Tp, Tm, Ts);
end

function exp = load_current_experiment(fname, swap_temp)
    data_table = readtable(fname);
    
    t = table2array(data_table(1:end, 1));
    i = table2array(data_table(1:end, 2));
    i2 = i.^2;

    Tp = table2array(data_table(1:end, 25)) - 23;
    Tm = table2array(data_table(1:end, 26)) - 23;
    
    Ts = mean(-t(1:end-1) + t(2:end));
    
    exp = iddata(Tm, i2, Ts);
end

function [e, z] = load_experiments(fname, swap_temp)
    data_table = readtable(fname);
    
    t = table2array(data_table(1:end, 1));
    i = table2array(data_table(1:end, 2));
    i2 = i.^2;

    if swap_temp
        Tp = table2array(data_table(1:end, 26)) - 23;
        Tm = table2array(data_table(1:end, 25)) - 23;
    else
        Tp = table2array(data_table(1:end, 25)) - 23;
        Tm = table2array(data_table(1:end, 26)) - 23;
    end
    
    Ts = mean(-t(1:end-1) + t(2:end))
    
    e = iddata(Tp, Tm, Ts);
    z = iddata(Tm, i2, Ts);
end

function [e, z] = load_experiments_alt(fname)
    data_table = readtable(fname,'PreserveVariableNames',true);
    headers = data_table.Properties.VariableNames;
    
    time_idx = find(ismember(headers,'time [s]'));
    i_idx = find(ismember(headers,'a1 q-axis cmd [A]'));
    i_idx = find(ismember(headers,'a1 q-axis [A]'));
    Tm_idx = find(ismember(headers,'motor temp [C]'));
    Tp_idx = find(ismember(headers,'housing temp [C]'));

    t = table2array(data_table(1:end, time_idx));
    i = table2array(data_table(1:end, i_idx));
    Tm = table2array(data_table(1:end, Tm_idx)) - 23;
    Tp = table2array(data_table(1:end, Tp_idx)) - 23;
    i2 = i.^2;
    
    Ts = mean(-t(1:end-1) + t(2:end))
    
    e = iddata(Tp, Tm, Ts);
    z = iddata(Tm, i2, Ts);
end