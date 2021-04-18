e1 = load_temp_experiment('futek_test_13_04_2021_20-10-36.csv');

NN2 = struc(1:3,1:3,1000);
% selstruc(arxstruc(e1(:,:,1),e1(:,:,1),NN2))

etf = tfest(e1,2,[],0)

e2 = load_temp_experiment('futek_test_14_04_2021_16-17-17.csv');
e3 = load_temp_experiment('futek_test_14_04_2021_16-13-41.csv');

compare(e1, etf)

%%
z1 = load_current_experiment('futek_test_13_04_2021_20-10-36.csv');
% NN2 = struc(1:3,1:3,10);
% selstruc(arxstruc(e1(:,:,1),e1(:,:,1),NN2))
% z1 = z1(1:5000);

ztf = tfest(z1,2,[],0)

z2 = load_current_experiment('futek_test_14_04_2021_16-17-17.csv');
z3 = load_current_experiment('futek_test_14_04_2021_16-13-41.csv');

compare(z1, ztf)

%%

z = z3;
e = e3;

[yfit, ~, ~] = compare(z, ztf);
e_hat = iddata(e.y, yfit.y, e.Ts);
[yfit, ~, ~] = compare(e_hat, etf);
c_hat = iddata(yfit.y, z.u, e.Ts);
c_true = iddata(e.y, z.u, e.Ts);

figure;
subplot(3,1,1)
plot(z.SamplingInstants, z.u, 'b-', 'DisplayName','i^2 motor input');
title('Motor Current Command')
legend('location','best');

subplot(3,1,2)
hold on
plot(e.SamplingInstants, e.u, 'k-', 'DisplayName', 'Motor Temp Meas')
plot(e_hat.SamplingInstants, e_hat.u, 'b--', 'DisplayName', 'Motor Temp Est')
title('Motor Temp')
legend('location','best');
hold off

subplot(3,1,3)
hold on
plot(c_true.SamplingInstants, c_true.y, 'k-', 'DisplayName','PLA Temp Meas')
plot(c_hat.SamplingInstants, c_hat.y, 'b--', 'DisplayName','PLA Temp Est')
title('PLA Temp')
legend('location','best');
hold off;

function exp = load_temp_experiment(fname)
    data_table = readtable(fname);
    
    t = table2array(data_table(1:end, 1));
    i = table2array(data_table(1:end, 2));
    i2 = i.^2;

    Tp = table2array(data_table(1:end, 25)) - 23;
    Tm = table2array(data_table(1:end, 26)) - 23;
    
    Ts = mean(-t(1:end-1) + t(2:end));
    
    exp = iddata(Tp, Tm, Ts);
end

function exp = load_current_experiment(fname)
    data_table = readtable(fname);
    
    t = table2array(data_table(1:end, 1));
    i = table2array(data_table(1:end, 2));
    i2 = i.^2;

    Tp = table2array(data_table(1:end, 25)) - 23;
    Tm = table2array(data_table(1:end, 26)) - 23;
    
    Ts = mean(-t(1:end-1) + t(2:end));
    
    exp = iddata(Tm, i2, Ts);
end