%% KT simple
datafile = "futek_test_11_05_2021_18-17-23.csv";
datafile = "futek_test_11_05_2021_19-28-26.csv";
datafile = "futek_data/futek_test_13_05_2021_18-10-31.csv";

data_table = readtable(datafile,'PreserveVariableNames',true);
headers = data_table.Properties.VariableNames;

time_idx = find(ismember(headers,'time [s]'));
a1_q_idx = find(ismember(headers,'a1 q-axis [A]'));
a2_q_idx = find(ismember(headers,'a2 q-axis [A]'));
a1_q_cmd_idx = find(ismember(headers,'a1 q-axis cmd [A]'));
a2_q_cmd_idx = find(ismember(headers,'a2 q-axis cmd [A]'));
ts_idx = find(ismember(headers,'trs605-5 torque [Nm]'));

time = table2array(data_table(1:end, time_idx));
a1_q = table2array(data_table(1:end, a1_q_idx));
a2_q = table2array(data_table(1:end, a2_q_idx));
a1_q_cmd = table2array(data_table(1:end, a1_q_cmd_idx));
a2_q_cmd = table2array(data_table(1:end, a2_q_cmd_idx));
ts = table2array(data_table(1:end, ts_idx));

Ts = median(abs(time - circshift(time, 1)));

buffer_time = 0.5;
buffer = round(buffer_time/Ts);

ss_mask = time >= 0;
for ii = 1:buffer
    ss_mask = ss_mask &...
        abs(a1_q_cmd - circshift(a1_q_cmd, ii)) < 0.01 & ...
        abs(a1_q_cmd - circshift(a1_q_cmd, -ii)) < 0.01 & ...
        abs(a2_q_cmd - circshift(a2_q_cmd, ii)) < 0.01 & ...
        abs(a2_q_cmd - circshift(a2_q_cmd, -ii)) < 0.01;
end
mean(ss_mask);
time = time(ss_mask);
a1_q = a1_q(ss_mask);
a2_q = a2_q(ss_mask);
a1_q_cmd = a1_q_cmd(ss_mask);
a2_q_cmd = a2_q_cmd(ss_mask);
ts = ts(ss_mask);

a1_driving_mask = abs(a1_q_cmd) > 0.01; a2_driving_mask = abs(a2_q_cmd) > 0.01;
idle_mask = (abs(a1_q_cmd) <= 0.01) & (abs(a2_q_cmd) <= 0.01);
% a1_driving_mask = a1_driving_mask | idle_mask;
% a2_driving_mask = a2_driving_mask | idle_mask;

ts_mean = mean(ts(idle_mask)); ts = ts - ts_mean;

q_ideal = unique(a1_q_cmd);

q11 = []; q12 = []; q21 = []; q22 = [];
ts11= []; ts12= []; ts21= []; ts22= [];
for jj = 1:length(q_ideal)
    q11 = [q11, mean(a1_q(a1_q_cmd==q_ideal(jj)))];
    q12 = [q12, mean(a1_q(a2_q_cmd==q_ideal(jj)))];
    q21 = [q21, mean(a2_q(a2_q_cmd==q_ideal(jj)))];
    q22 = [q22, mean(a2_q(a1_q_cmd==q_ideal(jj)))];
    
    ts11 = [ts11, mean(ts(a1_q_cmd==q_ideal(jj)))];
    ts12 = [ts12, mean(ts(a2_q_cmd==q_ideal(jj)))];
    ts21 = [ts21, mean(ts(a2_q_cmd==q_ideal(jj)))];
    ts22 = [ts22, mean(ts(a1_q_cmd==q_ideal(jj)))];
end
alpha = 0.1;
figure; hold on
subplot(1,2,1)
hold on;
s = scatter(a1_q(a1_driving_mask), ts(a1_driving_mask), 2, [.5 .5 .5],'HandleVisibility','off');
s.MarkerFaceAlpha = alpha; s.MarkerEdgeAlpha = alpha;
scatter(q11, ts11, 5, 'filled', 'k','HandleVisibility','off');
[p, x, ~, est] = polynom_fit(q11, ts11, 1);
plot(x([1 end]), est([1 end]), 'k',...
    'DisplayName',sprintf("Driving, $\\tau = %.3f i_q + %.3f$", p(1),p(2)));

s = scatter(a1_q(a2_driving_mask), ts(a2_driving_mask), 2, [.5 0 0],'HandleVisibility','off');
s.MarkerFaceAlpha = alpha; s.MarkerEdgeAlpha = alpha;
scatter(q12, ts12, 5, 'filled', 'r','HandleVisibility','off');
[p, x, ~, est] = polynom_fit(q12, ts12, 1);
plot(x([1 end]), est([1 end]), 'r',...
    'DisplayName',sprintf("Driven, $\\tau = %.3f i_q + %.3f$", p(1),p(2)));

ylabel('Torque ($\tau$) [Nm]'); xlabel('q-axis Current ($i_q$) [A]'); legend('location','southeast');
title('Actuator 1')
hold off;

subplot(1,2,2)
hold on;
s = scatter(a2_q(a2_driving_mask), ts(a2_driving_mask), 2, [.5 .5 .5],'HandleVisibility','off');
s.MarkerFaceAlpha = alpha; s.MarkerEdgeAlpha = alpha;
scatter(q21, ts21, 5, 'filled', 'k','HandleVisibility','off');
[p, x, ~, est] = polynom_fit(q21, ts21, 1);
plot(x([1 end]), est([1 end]), 'k',...
    'DisplayName',sprintf("Driving, $\\tau = %.3f i_q + %.3f$", p(1),p(2)));

s = scatter(a2_q(a1_driving_mask), ts(a1_driving_mask), 2, [.5 0 0],'HandleVisibility','off');
s.MarkerFaceAlpha = alpha; s.MarkerEdgeAlpha = alpha;
scatter(q22, ts22, 5, 'filled', 'r','HandleVisibility','off');
[p, x, ~, est] = polynom_fit(q22, ts22, 1);
plot(x([1 end]), est([1 end]), 'r',...
    'DisplayName',sprintf("Driven, $\\tau = %.3f i_q + %.3f$", p(1),p(2)));

ylabel('Torque ($\tau$) [Nm]'); xlabel('q-axis Current ($i_q$) [A]'); legend('location','southeast');
title('Actuator 2')
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
