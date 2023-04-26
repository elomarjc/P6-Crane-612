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

D_theta = (s + 1.5);

InnerLoop = feedback(G_theta, D_theta*H_theta);

G_x = -(l_P*s^2+ g)/s^2;



figure(1)
rlocus(minreal(InnerLoop*G_x))
%export_fig("xLoopRLocus.pdf")


D_x = (.5 + s);

figure(2)
hold on
polesCon1 = rlocus(minreal(InnerLoop*G_x*D_x),5);
%plot(real(polesCon1), imag(polesCon1),'^k')
rlocus(minreal(InnerLoop*G_x*D_x))
hold off

%export_fig("xLoopRLocus2.pdf")


figure(3258)
D_thetaMaxGain = D_theta*22;
InnerLoop = feedback(G_theta, D_thetaMaxGain*H_theta);
rlocus(minreal(InnerLoop*G_x*D_x))
%export_fig("xRlocusMaxGain.pdf")


figure(21468)
InnerLoop = feedback(G_theta, D_thetaMaxGain*H_theta);
D_xPID =(2 + 2*s + 4/8/s) ;
rlocus(minreal(InnerLoop*G_x*D_xPID))


figure(2853)
rlocus(minreal(G_theta*H_theta*D_theta))

figure(348)
D_thetaGain = D_theta*9;
polesInAngle = rlocus(minreal(G_theta*H_theta*D_theta),10);
plot(real(polesInAngle), imag(polesInAngle), '^k')
hold on
rlocus(minreal(G_theta*H_theta*D_theta))
hold off

%export_fig("angleRlocusDesignedGain.pdf")

figure(329853)
InnerLoop = feedback(G_theta, H_theta*D_thetaGain);
rlocus(minreal(InnerLoop*G_x)*D_x)
%export_fig("xRootLocusFinalInnerLoop.pdf")

figure(3293578)
InnerLoop = feedback(G_theta, H_theta*D_thetaGain);
polesOuterLoop1 = rlocus(minreal(InnerLoop*G_x)*D_x,5.5);
hold on
plot(real(polesOuterLoop1), imag(polesOuterLoop1), '^k')
rlocus(minreal(InnerLoop*G_x)*D_x)
hold off
%export_fig("xRootLocusInitCon.pdf")




%%
figure(2375)
simulationAngle = sim("angleControllerValidation.slx");
plot(simulationAngle.tout, simulationAngle.Theta1*180/pi)
xlim([0 10])
xlabel("Time [s]")
ylabel("Angle [deg]")
grid on
export_fig("angleControllerInit45Deg.pdf")

%%
xConP = .5;
xConD =1;
xConI = 0;
simXGain = 10;

%%
figure(984235)

%xConP = 4;
%xConD = 4;
%xConI = 4/8;
%simXGain = 7.5;
xConP = 1.5;
xConD =1.5;
xConI = 1.5/8;
simXGain = 22;
D_x = (xConP + xConD*s + xConI/s);
[Y, t] = step(feedback(D_x*simXGain*G_x*InnerLoop,1),20);
simulation = sim("x non linear model\CraneModel.slx");
plot(t,Y)
hold on
plot(simulation.tout-1, simulation.X1)
hold off
xlim([0 10])
grid on
yline(1+.064)
yline(1-.064)
legend("Linear model", "Non linear model", "Location","southeast")
xlabel("Time [s]")
ylabel("Position [m]")
export_fig("stepXController1NonAcceptableSim.pdf")


%%
figure
InnerLoop = feedback(G_theta, H_theta*D_theta*9);

polesRLocus = rlocus(minreal(InnerLoop*G_x*(1.5+1.5*s+3/8/s)),5.25);
hold on
plot(real(polesRLocus), imag(polesRLocus), '^k')
rlocus(minreal(InnerLoop*G_x*(1.5+1.5*s+3/8/s)))
hold off
export_fig("xRlocus3.pdf")




%%
figure
rlocus(minreal(InnerLoop*G_x*(1.4+1.4*s+6/8/s)))
%%
figure(43)
simXGain = 1;
noGainSimulation = sim("x non linear model\CraneModel.slx");
[tout, y] = step(feedback(InnerLoop*G_x*D_x*simXGain,1),15);
plot(y,tout)
hold on
plot(noGainSimulation.tout-1, noGainSimulation.X1)
hold off
errorBand = .064;
errorBandValue=(1)*errorBand;
yline(1 + errorBandValue)
yline(1 - errorBandValue)
grid on
xlim([0 15])
xticks(0:2:15)
legend("Linear model", "Non linear model", "Location", "south east")
xlabel("Time [s]")
ylabel("Trolley position [m]")
export_fig("xControllerStep1.pdf")

%%

figure(4353)
simXGain = 5;
noGainSimulation = sim("x non linear model\CraneModel.slx");    
[tout, y] = step(feedback(InnerLoop*G_x*D_x*simXGain,1),15);
%plot(y,tout)
hold on
plot(noGainSimulation.tout-1, noGainSimulation.X1)
hold off
errorBand = .07;
errorBandValue=(1)*errorBand;
yline(1 + errorBandValue)
yline(1 - errorBandValue)
grid on
xlim([0 15])
xticks(0:2:15)
%legend("Linear model", "Non linear model", "Location", "south east")
xlabel("Time [s]")
ylabel("Trolley position [m]")
export_fig("xControllerStep2.pdf")


figure(3504)
plot(noGainSimulation.tout-1, noGainSimulation.XContainer)
errorBand = .07;
errorBandValue=(1)*errorBand;
yline(1 + errorBandValue)
yline(1 - errorBandValue)
grid on
xlim([0 15])
xticks(0:2:15)
%legend("Linear model", "Non linear model", "Location", "south east")
xlabel("Time [s]")
ylabel("Container position [m]")
export_fig("xContainerControllerStep2.pdf")



%%
figure(3)
D_x = D_x*1.8*9;

xLoop = minreal(feedback(D_x*G_x*InnerLoop,1));
pzmap(xLoop)

figure(4)
step(xLoop)



function export_fig(name)
    x0=0;
    y0=0;
    plotwidth=650/1.5;
    plotHeight=400/1.5;
    set(gcf,'position',[x0,y0,plotwidth,plotHeight])
    
    
    exportgraphics(gcf,name,'ContentType','vector')
end

