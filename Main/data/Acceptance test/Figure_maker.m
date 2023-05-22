%% Acceptance test 2
clear figure
load('Acceptance test 2_data.mat');
figure
hold on % combines everything into one figure
plot(t,angle, 'red') % controller step response
line([0,36.11], [20*pi/180, 20*pi/180]); % [deg] -> [rad]
line([0,36.11], [-20*pi/180, -20*pi/180]);
xlabel('Time [s]');
ylabel('Angle [rad]');
title('Technical requirement 2 acceptance test')
legend('Angle', 'Max angle per tech. req. 2','location', 'southeast')
hold off

%% Acceptance test 3
clear figure
load('Acceptance test 3_data.mat');
hold on % combines everything into one figure
plot(t,Positionup, 'red') % controller step response
plot(t,Positiondown, 'blue') % controller step response
xlabel('Time [s]');
ylabel('Position [m]');
title('Technical requirement 3 acceptance test')
legend('Step response UP', 'Step response DOWN','Location', 'southeast')
hold off

%% Acceptance test 4
clear figure
load('Acceptance test 4_data.mat');
hold on % combines everything into one figure
plot(t,Position, 'red') % controller step response
xlabel('Time [s]');
ylabel('Position [m]');
title('Technical requirement 4 acceptance test')
legend('Step response X','Location', 'southeast')
hold off