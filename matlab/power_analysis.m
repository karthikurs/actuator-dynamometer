%% Dynamometer Power Analysis
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');

datafile = "futek_test_18_05_2021_14-50-07.csv";

data_table = readtable(datafile,'PreserveVariableNames',true);
headers = data_table.Properties.VariableNames;

time_idx = find(ismember(headers,'time [s]'));
a1_q_idx = find(ismember(headers,'a1 q-axis [A]'));
a1_v_idx = find(ismember(headers,'a1 velocity [rad/s]'));
a2_q_idx = find(ismember(headers,'a2 q-axis [A]'));
a1_q_cmd_idx = find(ismember(headers,'a1 q-axis cmd [A]'));
a2_q_cmd_idx = find(ismember(headers,'a2 q-axis cmd [A]'));
ts_idx = find(ismember(headers,'trs605-5 torque [Nm]'));
p1_idx = find(ismember(headers,'ina1 power [W]'));
p2_idx = find(ismember(headers,'ina2 power [W]'));
load_v_idx = find(ismember(headers,'load velocity cmd [Hz]'));

time = table2array(data_table(1:end, time_idx));
a1_q = table2array(data_table(1:end, a1_q_idx));
a1_v = table2array(data_table(1:end, a1_v_idx));
a2_q = table2array(data_table(1:end, a2_q_idx));
a1_q_cmd = table2array(data_table(1:end, a1_q_cmd_idx));
a2_q_cmd = table2array(data_table(1:end, a2_q_cmd_idx));
ts = table2array(data_table(1:end, ts_idx));
p1 = table2array(data_table(1:end, p1_idx));
p2 = table2array(data_table(1:end, p2_idx));
load_v = table2array(data_table(1:end, load_v_idx));

Ts = median(abs(time - circshift(time, 1)));

buffer_time = 0.375;
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
a1_v = a1_v(ss_mask);
a2_q = a2_q(ss_mask);
a1_q_cmd = a1_q_cmd(ss_mask);
a2_q_cmd = a2_q_cmd(ss_mask);
ts = ts(ss_mask);
p1 = p1(ss_mask);
p2 = p2(ss_mask);
load_v = load_v(ss_mask);

p_mech = ts.*a1_v*2*pi;
p_loss_a1 = p1 - p2 - p_mech;
p_loss_a2 = p_mech + p2;

q_cmds = unique(a1_q_cmd);
v_cmds = unique(load_v);

q_condition = [];
v_condition = [];
a1_eta = [];
a2_eta = [];

for ii = 1:length(q_cmds)
    for jj = 1:length(v_cmds)
        q_nom = q_cmds(ii);
        v_nom = v_cmds(jj);
        cond_mask = a1_q_cmd == q_nom & load_v == v_nom;
        if sum(cond_mask) == 0
            continue
        end
%         q_condition(end+1) = q_nom;
%         v_condition(end+1) = v_nom;
        q_condition(end+1) = mean(a1_q(cond_mask));
        v_condition(end+1) = mean(a1_v(cond_mask));
        a1_eta(end+1) = mean(p_mech(cond_mask))/(mean(p_mech(cond_mask)) + mean(p_loss_a1(cond_mask)));
        a2_eta(end+1) = (mean(p_mech(cond_mask)) - mean(p_loss_a1(cond_mask)))/mean(p_mech(cond_mask));
    end
end

a1_eta(a1_eta > 1) = 1;
a1_eta(a1_eta < -1) = -1;
a2_eta(a2_eta > 1) = 1;
a2_eta(a2_eta < -1) = -1;

pointsize = 50;

figure; hold on
subplot(1,2,1)
hold on
% plot3(q_condition.*.105, v_condition.*2.*pi, a1_eta, 'o');
scatter(v_condition.*2.*pi, q_condition.*.105, pointsize, a1_eta,'filled');
cb = colorbar();
title("Actuator 1 Electrical to Mechanical Efficiency");
ylabel("Nominal Torque, $\tau$ [Nm]")
xlabel("Nominal Speed, $\omega$ [rad/s]")
hold off
subplot(1,2,2)
hold on
% plot3(q_condition.*.105, v_condition.*2.*pi, a2_eta, 'o');
scatter(v_condition.*2.*pi, q_condition.*.105, pointsize, a2_eta,'filled');
cb = colorbar();
title("Actuator 2 Mechanical to Electrical Efficiency");
ylabel("Nominal Torque, $\tau$ [Nm]")
xlabel("Nominal Speed, $\omega$ [rad/s]")
hold off
hold off

% a1_driving_mask = abs(a1_q_cmd) > 0.01; a2_driving_mask = abs(a2_q_cmd) > 0.01;
% idle_mask = (abs(a1_q_cmd) <= 0.01) & (abs(a2_q_cmd) <= 0.01);