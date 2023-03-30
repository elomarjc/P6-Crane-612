data = readtable('test_data_1.csv');
%Time micros, pos, vel, pwm

timeFix = 1000000;

% Calculate current from pwm
current = (data.Var4-125.5)/10.2;

%% Plot current as function of time %%
figure(1)
plot(data.Var1/timeFix, current)

%% Plot pos as function of time %%
figure(2)
plot(data.Var1/timeFix, data.Var2)

%% Plot velocity as function of time %%
figure(3)
plot(data.Var1/timeFix, data.Var3)
