s = tf('s');

ts = 0.01; % Sample time

wc = fc*2*pi % Frequency to filter
wc_warp = 2/ts*tan(wc*ts/2) % Pre-warped frequency

z = 0.1; % Dampening factor

% Notch filter from lecture
D_notch = (s^2+s*2*z*wc_warp+wc_warp^2)/(s+wc_warp)^2;
D_notch_raw = (s^2+s*2*z*wc+wc^2)/(s+wc)^2;
Dz = c2d(D_notch,ts,'tustin')

wx = 5*2*pi; % Bandwidth

% Notch filter from : https://en.wikipedia.org/wiki/Band-stop_filter
D_notch_spike = (s^2+wc_warp^2)/(s^2+wx*s+wc_warp^2);
D_notch_spike_raw = (s^2+wc^2)/(s^2+wx*s+wc^2);
Dz_spike = c2d(D_notch_spike,ts,'tustin')

% Plots
margin(Dz)
hold on
margin(Dz_spike)
margin(D_notch_raw)
margin(D_notch_spike_raw)
grid on
