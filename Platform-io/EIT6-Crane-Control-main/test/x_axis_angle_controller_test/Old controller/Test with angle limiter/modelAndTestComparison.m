clear
close all
clc

run("simulation\variables.m");
simulation = sim("simulation\angleControllerValidation.slx");

data1 = readmatrix("step_1.csv");
data1t = (data1(:,1)-data1(1,1))/1000;
data2 = readmatrix("step_2.csv");
data2t = (data2(:,1)-data2(1,1))/1000;
data3 = readmatrix("step_3.csv");
data3t = (data3(:,1)-data3(1,1))/1000;
data4 = readmatrix("step_4.csv");
data4t = (data4(:,1)-data4(1,1))/1000;

simAngle = simulation.Theta1*180/pi;
plot(simulation.tout, simAngle)
hold on
plot(data1t, data1(:,4))
plot(data2t, data2(:,4))
plot(data3t, data3(:,4))
plot(data4t, data4(:,4))
hold off
legend("Simulation", "step_1", "step_2","step_3","step_4", 'Location','northwest')
xlim([0,10])
xlabel("Time [s]")
ylabel("Angle [deg]")

export_fig("testvsSimulation.pdf")


function export_fig(name)
    x0=0;
    y0=0;
    plotwidth=650;
    plotHeight=400;
    set(gcf,'position',[x0,y0,plotwidth,plotHeight])
    
    
    exportgraphics(gcf,name,'ContentType','vector')
end