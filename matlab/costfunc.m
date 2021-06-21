function summation = costfunc(X,targetMag,targetPhase,freq,Ts)

% This function provides the cost based on the sum of the absolute value of
% the difference between the potential second order system and the original
% estimated frequency response.
% The idea here is that the best fit 2nd order system will minimize the
% error between this system's frequency response and the original estimated
% frequency response

% Note: Magnitude or phase could be used to optimize--we typically use
% magnitude, which has better results.  To optimize over phase, change the
% weight W2
%
% Thanks to Ung Hee for assistance with this code


transfer_cont= tf(1,[X(1) X(2)]);          % define the function that you want to optimize
transfer = c2d(transfer_cont,Ts);               % convert it to discrete time

[mag,phase,wout] =bode(transfer,freq);          % obtain the magnitude/phase/frequency from bode
mag1 = squeeze(mag);               
phase1 = squeeze(phase);
wout1 = squeeze(wout);
wout1 = unwrap(wout1);


W2 = 0;                                         % weight of phase in objective function
W1 =1;                                          % weight of magnitude in objective function
summation= sum(W1*abs(mag1-targetMag)+W2*abs(phase1-targetPhase)); % objective function - this should be as close to zero as possible

% Return summation value
end