% ME 646, Winter 2021
% System Identification Examples
%
% Professor Rouse
% University of Michigan
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clc
clear

%% Power spectrum example with hidden sine wave 

Ts = 1/1000;                                            % Simulated sample interveal
Fs = 1/Ts;                                              % Simulated sample rate (Hz)
t_end = 5;                                              % Ending time
A1 = 5;                                                 % Amplitude (arbitrary units)
w1 = 100;                                               % Freqency of hidden sine wave (Hz)
time = 0:Ts:(t_end - Ts);
data = A1*sin((w1*time)*(2*pi))+40*rand(1,length(time));

window = 500;                                           % Number of samples to use in window
overlap = 400;                                          % Number of samples to overlap

[Pxx,f] = pwelch(detrend(data), window, overlap,[], Fs); % Create power spectrum using Welch method
figure
subplot(121)                                            % Plotting
plot(time,data, 'linewidth',2)                          % Plotting data in time domain
xlabel('time (s)')
ylabel('Amplitude')
axis([0 t_end -30 70])
subplot(122)
plot(f,mag2db(Pxx),'linewidth',2)                       % Plotting Auto Spectrum
xlabel('Frequency (Hz)')
ylabel('Power (dB/Hz)')

%% Example identification of a system without noise

% Model Transfer function estimate - creating second order system
wn = 80;
zeta = 0.6;

% System setup (time domain)
sys = tf(wn^2,[1 2*zeta*wn wn^2]);                      % Creates the transfer function
input = normrnd(0,5,1,10000);                           % White noise input (arbitary units, or could be N)
%input = smooth(input, 50);                             % Low pass filter with a 10 point moving average filter
t = 0:.001:(length(input)-1)*.001;                      % time vec (s)
Fs = 1000;                                              % Sample rate (Hz)
[output, t] = lsim(sys,input,t);                        % Simulates output, given the tf and the input

% System ID
window = 1000;                                          % Number of samples to use in window
overlap = 0;                                            % Number of samples to overlap

[EstH, EstF] = tfestimate(input, output, window, overlap, [], Fs);        % tfestimate determines the best transfer function
EstMag   = abs(EstH);                                   % Determines magnitude for complex number
EstPhase = angle(EstH)*(180/pi);                        % Estimatest the phase, convert to degrees
EstOmega = EstF*2*pi;                                   % Conversion of frequency axis to rad/s

% Unwrap phase (aesthetics)                             % Phase can wrap around 360 deg, and this loop unwraps around it--the net change to the phase is zero 
for i = 2:length(EstPhase)
    if EstPhase(i) - EstPhase(i-1) > 180
        EstPhase(i) = EstPhase(i) - 360;
    elseif EstPhase(i) - EstPhase(i-1) < -180
        EstPhase(i) = EstPhase(i) + 360;
    end
end


% Creation of actual frequency response
F = logspace(0, 3, 1000);                               % Creates log space variable
Omega = 2 * pi * F;                                     % Frequency vector (rad/s) 
[HMag, HPhase, HOmega] = bode(sys, Omega);              % uses BODE to determine the real TF
HMag   = squeeze(HMag);                                 % BODE outputs a 3D matrix, this collapses it
HPhase = squeeze(HPhase); 

figure                                                  % Plotting input and output in time domain
subplot(211)
plot(t,input,'linewidth',2)
xlabel('Time (s)')
ylabel('Input (AU)')
subplot(212)
plot(t,output,'linewidth',2)
xlabel('Time (s)')
ylabel('Output (AU)')


figure                                                  % Plotting in frequency domain
%  Magnitude plot on top
subplot(2,1,1)
semilogx(HOmega, mag2db(HMag), 'linewidth',2)
hold on
semilogx(EstOmega, mag2db(EstMag),'linewidth',2)
xlabel('\omega (rad/s)')
ylabel('|H| (dB)')
legend('Actual', 'Estimated')
%  Phase plot on bottom
subplot(2,1,2)
semilogx(HOmega, HPhase, 'linewidth',2)
hold on
semilogx(EstOmega, EstPhase, 'linewidth',2)
xlabel('\omega (rad/s)')
ylabel('Phase (^o)')

%% Example identification of a system with added noise

% Model Transfer function estimate ADDED NOISE
close all 
clear all
clc


% Model Transfer function estimate
wn = 80;
zeta = 0.6;

% Noise magnitude
input_amp = 1;                                         % variance of a normally distributed random variabe
output_amp = 2;                                         % variance of a normally distributed random variabe

% System setup (time domain)
Fs = 1000;                                              % Sample rate (Hz)
sys = tf(wn^2,[1 2*zeta*wn wn^2]);                      % Creates the transfer function
input = normrnd(0,5,1,10000);                           % White noise input (arbitary units, or could be N)
%input = smooth(input, 50);                             % Low pass filter with a 10 point moving average filter
t = 0:.001:(length(input)-1)*.001;                      % time vec (s)
[output, t] = lsim(sys,input,t);                             % Simulates output, given the tf and the input
outputNew = output+normrnd(0,output_amp,length(output),1);             % Adding output noise to system response, ouput
inputNew = input+normrnd(0,input_amp,1,length(input));  % Adding input noise on system input 

% System ID
window = 500;                                          % Number of samples to use in window
overlap = 100;                                            % Number of samples to overlap

[EstHn, EstFn] = tfestimate(inputNew, outputNew,window, overlap, [], Fs);        
EstMagn   = abs(EstHn);                                 % Determines magnitude for complex number
EstPhasen = angle(EstHn)*(180/pi);                      % Estimate the phase
EstOmegan = EstFn*2*pi;                                 % Conversion of frequency axis to rad/s

% Unwrap phase                                          % Phase can wrap around 360 deg, and this loop unwraps around it--the net change to the phase is zero
for i = 2:length(EstPhasen)
    if EstPhasen(i) - EstPhasen(i-1) > 180
        EstPhasen(i) = EstPhasen(i) - 360;
    elseif EstPhasen(i) - EstPhasen(i-1) < -180
        EstPhasen(i) = EstPhasen(i) + 360;
    end
end

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
F = logspace(0, 3, 1000);                               % Creates log space variable
Omega = 2 * pi * F;                                     % Frequency vector (rad/s) 
[HMag, HPhase, HOmega] = bode(sys, Omega);              % uses BODE to determine the real TF
HMag   = squeeze(HMag);                                 % BODE outputs a 3D matrix, this collapses it
HPhase = squeeze(HPhase); 


figure                                                      % Plotting
subplot(211)
plot(t,input,'linewidth',2)
hold on
plot(t,inputNew,'--','linewidth',2)
xlabel('Time (s)')
ylabel('Input (AU)')
legend('Original','Added Noise')
subplot(212)
plot(t,output,'linewidth',2)
hold on
plot(t,outputNew,'--','linewidth',2)
xlabel('Time (s)')
ylabel('Output (AU)')

figure                                                  % Plotting in frequency domain
%  Magnitude plot on top
subplot(2,1,1)
semilogx(HOmega, mag2db(HMag), 'linewidth',2)
hold on
semilogx(EstOmega, mag2db(EstMag),'linewidth',2)
semilogx(EstOmega, mag2db(EstMagn),'linewidth',2)

xlabel('\omega (rad/s)')
ylabel('|H| (dB)')
legend('Actual', 'Estimated','Est. + Noise')
%  Phase plot on bottom
subplot(2,1,2)
semilogx(HOmega, HPhase, 'linewidth',2)
hold on
semilogx(EstOmega, EstPhase, 'linewidth',2)
semilogx(EstOmega, EstPhasen, 'linewidth',2)
xlabel('\omega (rad/s)')
ylabel('Phase (^o)')

%% Second order identification from estimated frequency response

close all 
clear all
clc


% Model Transfer function estimate
wn = 80;
zeta = 0.6;

% Noise magnitude
input_amp = 1;
output_amp = .1;

% System setup (time domain)
Fs = 1000;                                              % Sample rate (Hz)
sys = tf(wn^2,[1 2*zeta*wn wn^2]);                      % Creates the transfer function
input = normrnd(0,5,1,10000);                           % White noise input (arbitary units, or could be N)
%input = smooth(input, 50);                             % Low pass filter with a 10 point moving average filter
t = 0:.001:(length(input)-1)*.001;                      % Time vec (s)
[output, t] = lsim(sys,input,t);                        % Simulates output, given the tf and the input
outputNew = output+normrnd(0,output_amp,length(output),1);             % Adding output noise to system response, ouput
inputNew = input+normrnd(0,input_amp,1,length(input));  % Adding input noise on system input 

% System ID
window = 500;                                          % Number of samples to use in window
overlap = 250;                                            % Number of samples to overlap

[EstHn, EstFn] = tfestimate(inputNew, outputNew,window, overlap, [], Fs);        
EstMagn   = abs(EstHn);                                 % Determines magnitude for complex number
EstPhasen = angle(EstHn)*(180/pi);                      % Estimate the phase
EstOmegan = EstFn*2*pi;                                 % Conversion of frequency axis to rad/s

% Unwrap phase                                          % Phase can wrap around 360 deg, and this loop unwraps around it--the net change to the phase is zero
for i = 2:length(EstPhasen)
    if EstPhasen(i) - EstPhasen(i-1) > 180
        EstPhasen(i) = EstPhasen(i) - 360;
    elseif EstPhasen(i) - EstPhasen(i-1) < -180
        EstPhasen(i) = EstPhasen(i) + 360;
    end
end

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
F = logspace(0, 3, 1000);                               % Creates log space variable
Omega = 2 * pi * F;                                     % Frequency vector (rad/s) 
[HMag, HPhase, HOmega] = bode(sys, Omega);              % Uses BODE to determine the real TF
HMag   = squeeze(HMag);                                 % BODE outputs a 3D matrix, this collapses it
HPhase = squeeze(HPhase); 

% Fitting 2nd Order system to the estimated frequency response using tfestimate
Ts = mean(diff(t));                                      % Get the sampling time
sysfun = @(X) costfunc(X,EstMagn,EstPhasen,EstOmega,Ts); % Define a cost function, putting estimated frequency with noise as a target
lb = [0 0 0];                                            % Lower bound of the constriant, does not permit negative coefficients (not physically possible)
ub =[];                                                  % Upper bound of the constriant    

x0 =[rand() rand() rand()];                                         % Initial value for the optimizer, random numbers in this case
options = optimoptions('fmincon','OptimalityTolerance',1e-10);      % Setting an agressive torelance to give better fitting
[result, fval] = fmincon(sysfun,x0,[],[],[],[],lb,ub,[],options);   % Result is the estimated parmeters. compare with true parameters

Hbk= tf(1,[result(1) result(2) result(3)]);              % Optimized 2nd order system


[EstHMag2nd, EstHPhase2nd, EstOmega2nd] = bode(Hbk, Omega);       % Uses BODE to determine the real TF
EstMag2nd  = squeeze(EstHMag2nd);                                 % BODE outputs a 3D matrix, this collapses it
EstPhase2nd = squeeze(EstHPhase2nd); 



figure                                                    % Plotting in frequency domain
%  Magnitude plot on top
subplot(2,1,1)
semilogx(HOmega, mag2db(HMag), 'linewidth',2)
hold on
semilogx(EstOmega, mag2db(EstMagn),'linewidth',2)
hold on
semilogx(EstOmega2nd, mag2db(EstMag2nd),'linewidth',2)

xlabel('\omega (rad/s)')
ylabel('|H| (dB)')
legend('Actual', 'Est. + Noise', '2nd order')

%  Phase plot on bottom
subplot(2,1,2)
semilogx(HOmega, HPhase, 'linewidth',2)
hold on
semilogx(EstOmega, EstPhasen, 'linewidth',2)
hold on
semilogx(EstOmega2nd, EstPhase2nd, 'linewidth',2)
xlabel('\omega (rad/s)')
ylabel('Phase (^o)')

sys_format = tf(1,[1/wn^2 2*zeta/wn 1]);                % Convert original system to format with 1 in the numerator (the system is identical)
True_system = sys_format                                % Print the true system that generated the data
Estimated_system = Hbk                                  % Print the identified system for comparison



