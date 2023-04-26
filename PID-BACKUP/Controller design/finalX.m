clear
close all
clc

s = tf('s');
run("x non linear model/variables.m")

l_P = InitWireLength;
b = k_ex/r_x;
a = M_1*l_P-m_l*l_P;
G_theta = -s/(s^3*a + s^2*B_x*l_P+s*g*M_1+g*B_x)*b;
H_theta = -1;

D_theta = (s + 1.5)*23;

InnerLoop = feedback(G_theta, D_theta*H_theta);


G_x = -(l_P*s^2+ g)/s^2;

%%

figure(1)
rlocus(minreal(InnerLoop*G_x))
%export_fig("xLoopRLocus.pdf")


xConP = .5;
xConI = 0;
xConD = 1;
D_x = (xConP + s*xConD);
simThetaGain = 23;

simXGain = 5;
figure(2)
hold on
polesCon1 = rlocus(minreal(InnerLoop*G_x*D_x),simXGain);
plot(real(polesCon1), imag(polesCon1),'^k')
rlocus(minreal(InnerLoop*G_x*D_x))
hold off
xlim([-8 .5])
grid on
%export_fig("xLoopRLocus2.pdf")



[y, t] = step(feedback(G_x*InnerLoop*D_x*simXGain,1),21);
simulation = sim("x non linear model\CraneModel.slx");
figure(22)
plot(t,y)
hold on 
plot(simulation.tout, simulation.X1)
yline(1+.05)
yline(1-.05)
hold off
xlim([0 20])
xlabel("Time [s]")
ylabel("Trolley position [m]")
legend("Linear model", "Non linear model", "Location","southeast")
grid on
%export_fig("xStepNonTuned.pdf")


%%
simThetaGain = 10;
InnerLoopLoweredGain = feedback(G_theta, H_theta*(1.5+s)*simThetaGain);
[y, t] = step(feedback(G_x*InnerLoopLoweredGain*D_x*simXGain,1),21);
simulation = sim("x non linear model\CraneModel.slx");
figure(23)
plot(t,y)
hold on 
plot(simulation.tout, simulation.X1)
yline(1+.05)
yline(1-.05)
hold off
xlabel("Time [s]")
ylabel("Trolley position [m]")
xlim([0 20])
grid on
legend("Linear model", "Non linear model", "Location","southeast")
export_fig("xStepNonTunedInnerGain10.pdf")

figure(232)
plot(simulation.tout, simulation.Theta1*180/pi, "Color",'#D95319')
xlabel("Time [s]")
ylabel("Angle [deg]")
xlim([0 20])
grid on
export_fig("xStepNonTunedInnerGain10Angle.pdf")

%%


%%
simThetaGain = 10;
InnerLoopLoweredGain = feedback(G_theta, H_theta*(1.5+s)*simThetaGain);
figure(3)
rlocus(InnerLoopLoweredGain*G_x)
ylim([-20 20])
%export_fig("xRlocusPID1.pdf")

d_xPID2 = (.5+ s)^2/s;
figure(322)
rlocus(InnerLoopLoweredGain*G_x*d_xPID2)
simXGain = 7.5;

figure(323)
polesAtGain = rlocus(InnerLoopLoweredGain*G_x*d_xPID2, simXGain);
rlocus(InnerLoopLoweredGain*G_x*d_xPID2)
hold on
plot(real(polesAtGain), imag(polesAtGain), "^k")
hold off
grid on
export_fig("xRlocusPID2.pdf")


figure(245)
step(feedback(G_x*InnerLoopLoweredGain*d_xPID2*simXGain,1))

% d_xPID1 = (.5+ s)*(.2 +s)/s;
% figure(32)
% rlocus(InnerLoopLoweredGain*G_x*d_xPID1)
% %export_fig("xRlocusPID2.pdf")

% d_xPID15 = (1.5 + 2.14*s + .21/s);
% figure(3215)
% polesAtGain = rlocus(InnerLoopLoweredGain*G_x*d_xPID15, 3);
% rlocus(InnerLoopLoweredGain*G_x*d_xPID15)
% hold on
% plot(real(polesAtGain), imag(polesAtGain), "^k")
% hold off
%export_fig("xRlocusPID2.pdf")


% d_xPID16 = (1.3 + 2.14*s + .21/s);
% figure(332)
% rlocus(InnerLoopLoweredGain*G_x*d_xPID16)
% figure(33)
% step(feedback(InnerLoop*d_xPID15*3*G_x,1))
% step(feedback(InnerLoop*d_xPID16*2.5*G_x,1))



%%
d_xPID2 = (.5+ s)^2/s;
figure(322)
rlocus(InnerLoopLoweredGain*G_x*d_xPID2)

d_xPID3 = (.2+ s)^2/s;
figure(323)
rlocus(InnerLoopLoweredGain*G_x*d_xPID3)

%%
% xConP = .6;
% xConD = 1.6;
% xConI = .4;
% simXGain = 4.5;   
xConP = 1;
xConD = 1;
xConI = .25;
simXGain = 7.5;  
simThetaGain = 10;


xConPLin = 1;
xConDLin = 1;
xConILin = .25;
simXGainLin = 7.5;   
linPIDVSSIM = (xConPLin + xConDLin*s + xConILin/s);

[y, t] = step(feedback(G_x*InnerLoopLoweredGain*linPIDVSSIM*simXGainLin,1),21);
simulation = sim("x non linear model\CraneModel.slx");
figure(225)
plot(t,y)
hold on 
plot(simulation.tout, simulation.X1)
yline(1+.05)
yline(1-.05)
hold off
xlabel("Time [s]")
ylabel("Trolley position [m]")
xlim([0 20])
grid on
legend("Linear model", "Non linear model", "Location","southeast")
%export_fig("NonTunedXcontrollerStep1Meter.pdf")



%%
xConP = .75;
xConD = 1;
xConI = .3;
simXGain = 6.5;  
simThetaGain = 8;

xConP = 1.8;
xConD = 1.65;
xConI = 2/8;
simXGain = 7.5;
simThetaGain = 8;



xConPLin = 1.8;
xConDLin = 1.65;
xConILin = 2/8;
simXGainLin = 7.5;   
linPIDVSSIM = (xConPLin + xConDLin*s + xConILin/s);
InnerLoopLoweredGain = feedback(G_theta, (2 +s)*simThetaGain*H_theta);


[y, t] = step(feedback(G_x*InnerLoopLoweredGain*linPIDVSSIM*simXGainLin,1),21);
simulation = sim("x non linear model\CraneModel.slx");
figure(22)
%plot(t,y)
hold on 
plot(simulation.tout, simulation.X1)
yline(1+.05)
yline(1-.05)
%yline(1+.5/3)

xlabel("Time [s]")
ylabel("Trolley position [m]")
hold off
xlim([0 20])
%legend("Linear model", "Non linear model", "Location","southeast")
grid on
export_fig("TunedXcontrollerStep1Meter.pdf")


figure(226)
plot(simulation.tout, simulation.Theta1*180/pi)
xlabel("Time [s]")
ylabel("Angle [deg]")
xlim([0 20])
grid on
export_fig("TunedXcontrollerStep1MeterAngle.pdf")



%%


xConP = 1.4;
xConD = 1.4;
xConI = 6/8;
simXGain = 5.5;
simThetaGain = 8;
hptau = 1;

simulation = sim("x non linear model\CraneModel.slx");

testData = readmatrix('x non linear model\testOfControllers\4-5-2022\PIDtest13.txt', 'CommentStyle', '//');
testTime = (testData(:,1)-testData(1,1))/1000;
testX = testData(:,2)-testData(1,2);
testY = testData(:,4);
testAngle = testData(:,5);
testXRef = testData(:,6)-testData(1,2);
testXcon = testData(:,9);
testAngleCon = testData(:,8);

figure(4)
plot(simulation.tout, simulation.X1)
hold on
plot(testTime, testX)
hold off
legend("Non linear model", "Measurement", "Location","south east")
xlabel("Time [s]")
ylabel("Position [m]")
ylim([-.2 4.2])
grid on


figure(42)
plot(simulation.tout, simulation.Theta1*180/pi)
hold on
plot(testTime, testAngle)
hold off
legend("Non linear model", "Measurement", "Location","south east")
xlabel("Time [s]")
ylabel("angle [deg]")
grid on

% figure(43)
% plot(simulation.conTime, simulation.Xcon+simulation.Anglecon)
% hold on
% stairs(testTime, testXcon+testAngleCon)
% hold off
% legend("Non linear model", "Measurement", "Location","south east")
% xlabel("Time [s]")
% ylabel("Current [A]")
% grid on
% 
% 
% figure(44)
% plot(simulation.conTime, simulation.Anglecon)
% hold on
% plot(testTime, testAngleCon)
% hold off
% legend("Non linear model", "Measurement", "Location","south east")
% xlabel("Time [s]")
% ylabel("angle [deg]")
% grid on



%%
D_xPID = (2 + 2*s +1/2/s);
figure(4)
hold on
polesCon1 = rlocus(minreal(InnerLoop*G_x*D_x),5);
%plot(real(polesCon1), imag(polesCon1),'^k')
rlocus(minreal(InnerLoop*G_x*D_xPID))
hold off

%%
figure(32)
polesCon1 = rlocus(minreal(InnerLoop*G_x*D_x),5);
%plot(real(polesCon1), imag(polesCon1),'^k')
rlocus(minreal(InnerLoop*G_x*(1+10/7*s + .14/s)))
hold off



D_xPID2 = (1.4 + 1.4*s + 6/8/s);
figure(4)
hold on
polesCon1 = rlocus(minreal(InnerLoop*G_x*D_xPID2),5);
%plot(real(polesCon1), imag(polesCon1),'^k')
rlocus(minreal(InnerLoop*G_x*D_xPID2))
hold off


figure(5)
rlocus(minreal(InnerLoop*G_x*(s+.25)))

figure(6)
rlocus(minreal(InnerLoop*G_x*(1+2*s+1/8/s)))

function export_fig(name)
    x0=0;
    y0=0;
    plotwidth=650/1.5;
    plotHeight=400/1.5;
    set(gcf,'position',[x0,y0,plotwidth,plotHeight])
    
    
    exportgraphics(gcf,name,'ContentType','vector')
end
