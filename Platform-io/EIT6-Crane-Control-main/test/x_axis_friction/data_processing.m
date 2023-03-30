%% Load csv files, gauge and video

guage_data_slow = readtable("x_friction_slow.csv");
video_data_slow = readtable("x_friction_slow_video.csv");

guage_data_fast = readtable("x_friction_fast.csv");
video_data_fast = readtable("x_friction_fast_video.csv");

% It turns out friction can vary depending on speed, both increasing and
% decreasing. Depends on the material properties, and for decreasing speeds
% it might be due to a phenomenon called stick-slip, where the surfaces are
% quickly switching between static and dynamic friction.

% "Stick-slip phenomena are associated with a decrease in the COF as the sliding
% speed increases. The transition from static to sliding friction also can be
% regarded as a case where the COF decreases as sliding speed increases."

% http://www.physics.usyd.edu.au/~cross/PUBLICATIONS/30.%20FrictionvsSpeed.pdf

%%

slow_interp_time = 0:0.01:18;

% Guage
guage_data_slow_time = guage_data_slow.Var1-14;
guage_data_slow_force = guage_data_slow.Var2*9.82;

slow_force_interp = interp1(guage_data_slow_time,guage_data_slow_force,slow_interp_time);

% Video
video_slow_pos = sqrt(video_data_slow.x.^2 + video_data_slow.y.^2);
video_slow_vel = diff(video_slow_pos)./diff(video_data_slow.t);
video_slow_time = video_data_slow.t(1:length(video_data_slow.t)-1,1);

slow_vel_interp = interp1(video_slow_time,movmean(video_slow_vel,2),slow_interp_time);

% Calculate friction as b = f/vel
slow_fric = slow_force_interp./slow_vel_interp;

% % PLOT
% clf(gcf)
% hold on
% grid on
% plot(slow_interp_time, movmean(slow_fric,10))
% xlim([5 16])

% plot(slow_interp_time,slow_force_interp,slow_interp_time,slow_vel_interp*100)

%%

fast_interp_time = 0:0.01:9;

% Guage
guage_data_fast_time = guage_data_fast.Var1-3;
guage_data_fast_force = guage_data_fast.Var2*9.82;

fast_force_interp = interp1(guage_data_fast_time,guage_data_fast_force,fast_interp_time);

% Video
video_fast_pos = sqrt(video_data_fast.x.^2 + video_data_fast.y.^2);
video_fast_vel = diff(video_fast_pos)./diff(video_data_fast.t);
video_fast_time = video_data_fast.t(1:length(video_data_fast.t)-1,1);

fast_vel_interp = interp1(video_fast_time,movmean(video_fast_vel,2),fast_interp_time);

% Calculate friction as b = f/vel
fast_fric = fast_force_interp./fast_vel_interp;

% % PLOT
% clf(gcf)
% hold on
% grid on
% plot(fast_interp_time, movmean(fast_fric,10))
% xlim([4 9])

% plot(fast_interp_time,fast_force_interp,fast_interp_time,fast_vel_interp*100)

%% Plot of non-compensated and compensated dampening

slow_fric = ((slow_force_interp)./slow_vel_interp);
fast_fric = ((fast_force_interp)./fast_vel_interp);

figure('position',[0,0,1000,400])
tiledlayout(1,2)

nexttile
plot(slow_interp_time, movmean(slow_fric,200))
hold on
grid on
plot(fast_interp_time, movmean(fast_fric,200))
xlim([5 9])
ylim([0 200])
title("(a) Before constant compensation")
xlabel("Time [s]")
ylabel("Dampening constant [ks/s]")
legend("High velocity test","Low velocity test")

% Initialized with higher values
FBs = 100;
minFBs = 1000;
leastsquare = 1000;

% Optimized for the lowest squared error when calculating FBs
for i=10:0.1:20
    FBs = i;

    slow_fric = ((slow_force_interp-FBs)./slow_vel_interp);
    fast_fric = ((fast_force_interp-FBs)./fast_vel_interp);

    mvmnslow = movmean(slow_fric,100);
    mvmnfast = movmean(fast_fric,100);
    
    squarederror = sum((mvmnslow(500:900)-mvmnfast(500:900)).^2)/length(mvmnslow(500:900));
    if squarederror < leastsquare
        leastsquare = squarederror;
        minFBs = i;
    end

    if i == 20
        slow_fric = ((slow_force_interp-minFBs)./slow_vel_interp);
        fast_fric = ((fast_force_interp-minFBs)./fast_vel_interp);
        mvmnslow = movmean(slow_fric,100);
        mvmnfast = movmean(fast_fric,100);
        dampB = (sum(mvmnslow(500:900))+sum(mvmnfast(500:900)))/(length(mvmnfast(500:900))*2);

    end

end

disp("Viscous friction constant B : "+string(dampB))
disp("Coulob friction constant Fc : "+string(minFBs))
slow_fric = ((slow_force_interp-minFBs)./slow_vel_interp);
fast_fric = ((fast_force_interp-minFBs)./fast_vel_interp);

nexttile

plot(slow_interp_time, movmean(slow_fric,200))
hold on
grid on
plot(fast_interp_time, movmean(fast_fric,200))
xlim([5 9])
ylim([0 50])
title("(b) After constant compensation")
xlabel("Time [s]")
ylabel("Dampening constant [ks/s]")
legend("High velocity test","Low velocity test")

% exportgraphics(gcf,"friction_coeffecient_x_axis.pdf","ContentType","vector")

%% Calculate average dampening constant after compensation

mvmnslow = movmean(slow_fric,200);
mvmnfast = movmean(fast_fric,200);

figure(2)
plot(slow_interp_time(500:900), mvmnslow(500:900))
hold on
grid on
plot(fast_interp_time(500:900), mvmnfast(500:900))

avgval = (sum(mvmnslow(500:900))+sum(mvmnfast(500:900)))/(length(mvmnfast(500:900))*2)

xlim([5 9])
ylim([0 50])
% close gcf

%% Converting friction constant to constant current offset

clc

Fx = 17;
rx = 75e-3;
Kx = 0.609;
Ix = (Fx*rx)/(Kx);
disp("x-axis current offset - Ix : " + string(Ix))

Fy = 30; % guess for now
ry = 48e-3;
Ky = 0.710;
Iy = (Fy*ry)/(Ky);
disp("y-axis current offset - Iy : " + string(Iy))

%% Plot of force as a function of time

clf(gcf)

hold on
grid on

plot(guage_data_fast.Var1,guage_data_fast.Var2*9.82)
plot(guage_data_slow.Var1-10,guage_data_slow.Var2*9.82)

xlim([5 24])

%% Handy little function

function p = curr2pwm(current)
    if current > 10; current = 10; end
    if current < -10; current = -10; end
    p = 10.2*current+127.5;
end