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


%% Direct Viscous Damping

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

%% cpp validation

datafile = "dynamometer-data/dynamometer_test_07_06_2021_14-57-55.csv";
datafile = "dynamometer-data/dynamometer_test_07_06_2021_15-25-44.csv";
datafile = "dynamometer-data/dynamometer_test_07_06_2021_15-23-56.csv";

[Ts, t_exp] = load_grp_cpp_exp(datafile);

w = logspace(0,log10(pi/Ts),1024);

t_spa = spa(t_exp);
t_spa10 = spa(t_exp, 10, w);
t_spa100 = spa(t_exp, 100, w);
t_spa1000 = spa(t_exp, 1000, w);

ttf0 = tfest(t_exp,1,0,0);
ttf2 = tfest(t_exp,2,0,0);
% ttf3 = tfest(t_exp,3,0,0);

[Yfit, ~, ~] = compare(t_exp, ttf0);
t0_vaf = vaf(t_exp.OutputData, Yfit.OutputData)
[Yfit, ~, ~] = compare(t_exp, ttf2);
t2_vaf = vaf(t_exp.OutputData, Yfit.OutputData)
% [Yfit, ~, ~] = compare(t_exp, ttf3);
% t3_vaf = vaf(t_exp.OutputData, Yfit.OutputData)
hold on;
figure;
bodeplot(t_spa, t_spa10, t_spa100, t_spa1000,ttf0,ttf2);
legend('spa','spa 10','spa 100','spa 1000',...
    sprintf('first order (%.2f pc)',t0_vaf),...
    sprintf('second order (%.2f pc)',t2_vaf));
title(datafile);
hold off;

%% Second order identification from estimated frequency response

format compact
% clc
datafile = "dynamometer-data/dynamometer_test_07_06_2021_15-23-56.csv";

shaft_datafiles = ["dynamometer-data/dynamometer_test_22_06_2021_12-27-48.csv",...
    "dynamometer-data/dynamometer_test_22_06_2021_12-33-14.csv",...
    "dynamometer-data/dynamometer_test_22_06_2021_12-36-41.csv",...
    "dynamometer-data/dynamometer_test_22_06_2021_12-38-46.csv",...
    "dynamometer-data/dynamometer_test_22_06_2021_12-40-56.csv"];
ri50_no_gb_datafiles = ["dynamometer-data/dynamometer_test_21_06_2021_15-54-26.csv",...
    "dynamometer-data/dynamometer_test_21_06_2021_16-08-24.csv"];
ri50_gb_datafiles = ["dynamometer-data/dynamometer_test_21_06_2021_16-22-42.csv",...
    "dynamometer-data/dynamometer_test_22_06_2021_11-52-06.csv",...
    "dynamometer-data/dynamometer_test_22_06_2021_11-59-35.csv",...
    "dynamometer-data/dynamometer_test_22_06_2021_12-05-03.csv"];

ex8_no_gb_datafiles = ["dynamometer-data/dynamometer_test_23_06_2021_12-34-39.csv",...
    "dynamometer-data/dynamometer_test_23_06_2021_12-42-49.csv",...
    "dynamometer-data/dynamometer_test_23_06_2021_12-47-28.csv",...
    "dynamometer-data/dynamometer_test_23_06_2021_13-48-01.csv",...
    "dynamometer-data/dynamometer_test_23_06_2021_13-52-08.csv"];
ex8_gb_datafiles = ["dynamometer-data/dynamometer_test_23_06_2021_15-34-43.csv"];


% [Ts, t_exp] = load_grp_cpp_exp(datafile);
figure 
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');

subplot(1,5,1)
[J_shaft, B_shaft] = process_and_plot(shaft_datafiles, "Shaft Hardware");

subplot(1,5,2)
[J_ri50_nogb, B_ri50_nogb] = process_and_plot(ri50_no_gb_datafiles, "RI50 No GB");
subplot(1,5,3)
[J_ri50_gb, B_ri50_gb] = process_and_plot(ri50_gb_datafiles, "RI50 6-1 GB");

subplot(1,5,4)
[J_ex8_nogb, B_ex8_nogb] = process_and_plot(ex8_no_gb_datafiles, "EX8 No GB");
subplot(1,5,5)
[J_ex8_gb, B_ex8_gb] = process_and_plot(ex8_gb_datafiles, "EX8 6-1 GB");

sgt = sgtitle(sprintf("RI50: $J_{rotor}$ = %.3e kgm$^2$; $J_a$ = %.3e kgm$^2$, EX8: $J_{rotor}$ = %.3e kgm$^2$; $J_a$ = %.3e kgm$^2$",...
    J_ri50_nogb-J_shaft, J_ri50_gb-J_shaft, J_ex8_nogb-J_shaft, J_ex8_gb-J_shaft));
sgt.Interpreter = 'latex';

function [J, B] = process_and_plot(datafiles, title_str)
    [Ts, t_exp] = load_grp_cpp_exp(datafiles);
    [Hbk, Est, Fit, t, output, outputsim, J, B] = id_sys(Ts, t_exp);
    EstOmega = Est{1}; EstMag = Est{2}; EstPhase = Est{3};
    FitOmega = Fit{1}; FitMag = Fit{2}; FitPhase = Fit{3};
    fit_vaf = vaf(output, outputsim);
    
    semilogx(EstOmega./(2*pi), mag2db(EstMag))
    hold on
    semilogx(FitOmega./(2*pi), mag2db(FitMag))
    title(sprintf("%s, vaf = %2.2f\\%%",title_str, fit_vaf));
    xlabel('$f$ (Hz)')
    ylabel('Gain (dB)')
    legend('Data', '1st order fit')
    hold off
end

function [Hbk, Est, Fit, t, output, outputsim, J, B] = id_sys(Ts, t_exp)

Fs = 1/Ts;
input = t_exp.InputData;
output = t_exp.OutputData;

sprintf("data length = %d",length(input))

% System ID
window = 50000;                                          % Number of samples to use in window
overlap = 20000;                                            % Number of samples to overlap

[EstH, EstF] = tfestimate(input, output, window, overlap, [], Fs);
EstMag   = abs(EstH);                                   % Determines magnitude for complex number
EstPhase = angle(EstH)*(180/pi);                        % Estimate the phase
EstOmega = EstF*2*pi;                                   % Conversion of frequency axis to rad/s

% Unwrap phase                                          % Phase can wrap around 360 deg, and this loop unwraps around it--the net change to the phase is zero
for i = 2:length(EstPhase)
    if EstPhase(i) - EstPhase(i-1) > 100
        EstPhase(i) = EstPhase(i) - 360;
    elseif EstPhase(i) - EstPhase(i-1) < -100
        EstPhase(i) = EstPhase(i) + 360;
    end
end

% Creation of actual frequency response
F = logspace(log10(2*window/length(input)), log10(1/Ts), 1000);                               % Creates log space variable
Omega = 2 * pi * F;                                     % Frequency vector (rad/s) 

% Fitting 2nd Order system to the estimated frequency response using tfestimate
% Ts = mean(diff(t));                                      % Get the sampling time
sysfun = @(X) costfunc(X,EstMag,EstPhase,EstOmega,Ts); % Define a cost function, putting estimated frequency with noise as a target
lb = [0 0];                                            % Lower bound of the constriant, does not permit negative coefficients (not physically possible)
ub =[];                                                  % Upper bound of the constriant    

x0 =[2.636e-14+0.01*rand() 0.0001186+0.01*rand() 0.0233+0.01*rand()];
x0 =[rand() rand() rand()];
x0 =[0.01 0.01];% Initial value for the optimizer, random numbers in this case
options = optimoptions('fmincon','OptimalityTolerance',1e-15);      % Setting an agressive torelance to give better fitting
[result, fval] = fmincon(sysfun,x0,[],[],[],[],lb,ub,[],options);   % Result is the estimated parmeters. compare with true parameters

Hbk= tf(1,[result(1) result(2)]);              % Optimized 1st order system
J = result(1); B = result(2);

[FitHMag, FitHPhase, FitOmega] = bode(Hbk, Omega);       % Uses BODE to determine the real TF
FitMag  = squeeze(FitHMag);                                 % BODE outputs a 3D matrix, this collapses it
FitPhase = squeeze(FitHPhase); 

[outputsim, t] = lsim(Hbk,input,t_exp.SamplingInstants);
% outputsim = -outputsim;
fit_vaf1 = vaf(output, outputsim);
fit_vaf2 = vaf(output, -outputsim);

if fit_vaf2 > fit_vaf1
    outputsim = -outputsim;
end

Est = {EstOmega, EstMag, EstPhase};
Fit = {FitOmega, FitMag, FitPhase};
end

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

function [Ts, t_exp] = load_grp_cpp_exp(datafiles)
    time_oall = [];
    a1_v_oall = [];
    ts_oall = [];
    t_base = 0;
    for ii = 1:length(datafiles)
        datafile = datafiles(ii);
        data_table = readtable(datafile,'PreserveVariableNames',true,...
            'Delimiter',',');
        headers = data_table.Properties.VariableNames;

        time_idx = find(ismember(headers,'time [s]'));
        a1_v_idx = find(ismember(headers,'a1 velocity [rad/s]'));
        ts_idx = find(ismember(headers,'trs605-5 torque [Nm]'));

        time = table2array(data_table(1:end, time_idx));
        a1_v = table2array(data_table(1:end, a1_v_idx));
        ts = table2array(data_table(1:end, ts_idx));
        
        time_oall = [time_oall; time+t_base];
        a1_v_oall = [a1_v_oall; a1_v];
        ts_oall = [ts_oall; ts];
        t_base = time_oall(length(time_oall));
    end

    dt = abs(time - circshift(time, 1));
    Ts = median(dt(1:90)); % accounting for stupid mistake when taking data

    t_exp = iddata(a1_v_oall, ts_oall, Ts);
end