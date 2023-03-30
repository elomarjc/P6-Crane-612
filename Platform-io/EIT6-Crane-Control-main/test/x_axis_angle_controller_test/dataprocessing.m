clear
close all
clc

run("variables.m")
InitWireLength = .1;
simulation = sim("angleControllerValidation.slx");
figure(1)
plotData("10 centimeter", simulation)

export_fig("AngleTestNoNotch10cm.pdf")



InitWireLength = .5;
simulation = sim("angleControllerValidation.slx");
figure(2)
plotData("50 centimeter", simulation)
export_fig("AngleTestNoNotch50cm.pdf")


InitWireLength = 1;
simulation = sim("angleControllerValidation.slx");
figure(3)
plotData("1 meter", simulation)
export_fig("AngleTestNoNotch1m.pdf")


function plotData(file, simu)
    data1 = readmatrix(strcat(file, "\test1.csv"));
    data2 = readmatrix(strcat(file, "\test2.csv"));
    data3 = readmatrix(strcat(file, "\test3.csv"));
    
    t1 = (data1(:,1)-data1(1,1))/1000;
    t2 = (data2(:,1)-data2(1,1))/1000;
    t3 = (data3(:,1)-data3(1,1))/1000;
        
    plot(t1, data1(:,4))
    hold on
    plot(t2, data2(:,4))
    plot(t3, data3(:,4))
    plot(simu.tout, simu.Theta1*180/pi)
    hold off
    xlabel("Time [s]")
    ylabel("Angle [deg]")
    xlim([0, 10])

    
    legend("test1", "test2", "test3", "non linear model","Location","southeast")
end

function export_fig(name)
    x0=0;
    y0=0;
    plotwidth=650/1.5;
    plotHeight=400/1.5;
    set(gcf,'position',[x0,y0,plotwidth,plotHeight])
    
    
    exportgraphics(gcf,name,'ContentType','vector')
end