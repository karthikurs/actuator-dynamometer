%% motor temp to PLA temp model estimation
% e1 = load_temp_experiment('futek_test_13_04_2021_20-10-36.csv');
swap_temps=false;
[e1, z1] = load_experiments('futek_test_13_04_2021_20-10-36.csv', swap_temps);
% [e1, z1] = load_experiments('futek_test_13_05_2021_18-10-31.csv', swap_temps);

NN2 = struc(1:3,1:3,1000);
% selstruc(arxstruc(e1(:,:,1),e1(:,:,1),NN2))

etf = tfest(e1,1,0,0)

% e2 = load_temp_experiment('futek_test_14_04_2021_16-17-17.csv');
% e3 = load_temp_experiment('futek_test_14_04_2021_16-13-41.csv');
[e2, z2] = load_experiments('futek_test_14_04_2021_16-17-17.csv', swap_temps);
[e3, z3] = load_experiments('futek_test_14_04_2021_16-13-41.csv', swap_temps);

% compare(e3, etf)

%% current cmd to motor temp estimation
% z1 = load_current_experiment('futek_test_13_04_2021_20-10-36.csv');
% NN2 = struc(1:3,1:3,10);
% selstruc(arxstruc(e1(:,:,1),e1(:,:,1),NN2))
% z1 = z1(1:5000);

tfopt = tfestOptions('InitialCondition','estimate');
ztf = tfest(z1,1,0,0)

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

% plot_thermal_model('futek_test_27_04_2021_15-46-08.csv', etf, ztf, true, true);

plot_thermal_model('futek_test_13_05_2021_18-10-31.csv', etf, ztf, true, true);

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
    data_table = readtable(fname,'PreserveVariableNames',true);
    headers = data_table.Properties.VariableNames;

    time_idx = find(ismember(headers,'time [s]'));
    a1_q_idx = find(ismember(headers,'a1 q-axis [A]'));
    
    t = table2array(data_table(1:end, time_idx));
    i = table2array(data_table(1:end, a1_q_idx));
    i2 = i.^2;
    
    Tp_idx = find(ismember(headers,'housing temp [C]'));
    Tm_idx = find(ismember(headers,'motor temp [C]'));

    if swap_temp
        Tp = table2array(data_table(1:end, 26)) - 23;
        Tm = table2array(data_table(1:end, 25)) - 23;
    else
        Tp = table2array(data_table(1:end, 25)) - 23;
        Tm = table2array(data_table(1:end, 26)) - 23;
    end
    
    Tp = table2array(data_table(1:end, Tp_idx)) - 23;
    Tm = table2array(data_table(1:end, Tm_idx)) - 23;
    
    Ts = median(-t(1:end-1) + t(2:end))
    
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