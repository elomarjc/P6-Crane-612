%% Import data from csv files
Con3A=readtable('Tracker\3A2sekCon');
Tro3A=readtable('Tracker\3A2sekTro');
test3A = table(Con3A.t, Con3A.x, Con3A.y, Tro3A.x, Tro3A.y, 'VariableNames',{'time','xContainer','yContainer','xTrolley','yTrolley'});
test3A = [test3A  table(test3A.yContainer-test3A.yTrolley,'VariableNames',{'wireLength'}) table(test3A.xTrolley-test3A.xContainer,'VariableNames',{['xOffset']})];
test3A = [test3A  table(asin(test3A.xOffset./(test3A.wireLength+.13)),'VariableNames',{'theta'}) table(atan(test3A.xOffset./test3A.wireLength)*360/2/pi,'VariableNames',{'thetaDeg'})];
clear Con3A
clear Tro3A

Con5A=readtable('Tracker\5A2sekCon');
Tro5A=readtable('Tracker\5A2sekTro');
Tro5A=Tro5A(4:end,:);
test5A = table(Con5A.t, Con5A.x, Con5A.y, Tro5A.x, Tro5A.y, 'VariableNames',{'time','xContainer','yContainer','xTrolley','yTrolley'});
test5A = [test5A  table(test5A.yContainer-test5A.yTrolley,'VariableNames',{'wireLength'}) table(test5A.xTrolley-test5A.xContainer,'VariableNames',{['xOffset']})];
test5A = [test5A  table(atan(test5A.xOffset./(test5A.wireLength+.13)),'VariableNames',{'theta'}) table(atan(test5A.xOffset./test5A.wireLength)*360/2/pi,'VariableNames',{'thetaDeg'})];
clear Con5A
clear Tro5A


Con7A=readtable('Tracker\7A1sekCon');
Tro7A=readtable('Tracker\7A1sekTro');
test7A = table(Con7A.t, Con7A.x, Con7A.y, Tro7A.x, Tro7A.y, 'VariableNames',{'time','xContainer','yContainer','xTrolley','yTrolley'});
test7A = [test7A  table(test7A.yContainer-test7A.yTrolley,'VariableNames',{'wireLength'}) table(test7A.xTrolley-test7A.xContainer,'VariableNames',{['xOffset']})];
test7A = [test7A  table(atan(test7A.xOffset./(test7A.wireLength+.13)),'VariableNames',{'theta'}) table(atan(test7A.xOffset./test7A.wireLength)*360/2/pi,'VariableNames',{'thetaDeg'})];
clear Con7A
clear Tro7A

Con9A=readtable('Tracker\9A1sekCon');
Tro9A=readtable('Tracker\9A1sekTro');
test9A = table(Con9A.t, Con9A.x, Con9A.y, Tro9A.x, Tro9A.y, 'VariableNames',{'time','xContainer','yContainer','xTrolley','yTrolley'});
test9A = [test9A  table(test9A.yContainer-test9A.yTrolley,'VariableNames',{'wireLength'}) table(test9A.xTrolley-test9A.xContainer,'VariableNames',{['xOffset']})];
test9A = [test9A  table(atan(test9A.xOffset./(test9A.wireLength+.13)),'VariableNames',{'theta'}) table(atan(test9A.xOffset./test9A.wireLength)*360/2/pi,'VariableNames',{'thetaDeg'})];
clear Con9A
clear Tro9A

Con5AWC=readtable('Tracker\5A2sekWCCon');
Tro5AWC=readtable('Tracker\5A2sekWCTro');
test5AWC = table(Con5AWC.t, Con5AWC.x, Con5AWC.y, Tro5AWC.x, Tro5AWC.y, 'VariableNames',{'time','xContainer','yContainer','xTrolley','yTrolley'});
test5AWC = [test5AWC  table(test5AWC.yContainer-test5AWC.yTrolley,'VariableNames',{'wireLength'}) table(test5AWC.xTrolley-test5AWC.xContainer,'VariableNames',{['xOffset']})];
test5AWC = [test5AWC  table(atan(test5AWC.xOffset./(test5AWC.wireLength+.13)),'VariableNames',{'theta'}) table(atan(test5AWC.xOffset./test5AWC.wireLength)*360/2/pi,'VariableNames',{'thetaDeg'})];
clear Con5AWC
clear Tro5AWC

Con7A0k5=readtable('Tracker\0k5m7A1sekCon');
Tro7A0k5=readtable('Tracker\0k5m7A1sekTro');
test7A0k5 = table(Con7A0k5.t, Con7A0k5.x, Con7A0k5.y, Tro7A0k5.x, Tro7A0k5.y, 'VariableNames',{'time','xContainer','yContainer','xTrolley','yTrolley'});
test7A0k5 = [test7A0k5  table(test7A0k5.yContainer-test7A0k5.yTrolley,'VariableNames',{'wireLength'}) table(test7A0k5.xTrolley-test7A0k5.xContainer,'VariableNames',{['xOffset']})];
test7A0k5 = [test7A0k5  table(atan(test7A0k5.xOffset./(test7A0k5.wireLength+.13)),'VariableNames',{'theta'}) table(atan(test7A0k5.xOffset./test7A0k5.wireLength)*360/2/pi,'VariableNames',{'thetaDeg'})];
clear Con7A0k5
clear Tro7A0k5

disp('Measured data initialized')


%% 

%Model for x fungerer godt med 
%J_x=0.13
%B_x=12
%sFrictionX=20
%cFrictionX=18

runNSim=1;
runOSim=1;
exportPlots=1;

    %No movement of L direction
    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=0;

if(runOSim)
    %Simulation values before adjustment
    J_l = 0.14;
    J_x = 0.20;
    g = 9.82;
    k_el = 0.71;
    k_ex = 0.609;
    m_l = 3.89;
    m_t = 0.729;
    r_l = 0.05;
    r_x = 0.075;
    B_l = 9.4;
    B_x = 12.7;
    sFrictionX=20;
    cFrictionX=17.6;
    sFrictionL=50;
    cFrictionL=17.4;
    InitWireLength=1;

    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=3;
    simO3A=sim("CraneModel.slx");
    disp("Old Sim 1 done")

    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=5;
    simO5A=sim("CraneModel.slx");
    disp("Old Sim 2 done")

    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=7;
    simO7A=sim("CraneModel.slx");
    disp("Old Sim 3 done")

    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=9;
    simO9A=sim("CraneModel.slx");
    disp("Old Sim 4 done")

    m_l = 0.95;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=5;
    simO5AWC=sim("CraneModel.slx");
    disp("Old Sim 5 done")

    m_l = 3.89;
    InitWireLength=0.5;
    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=7;
    simO7A0k5=sim("CraneModel.slx");
    disp("Old Sim 6 done")
    disp("Old Sim done")

end






J_l = 0.14;
J_x = 0.13;
g = 9.82;
k_el = 0.71;
k_ex = 0.609;
m_l = 3.89;
m_t = 0.729;
r_l = 0.05;
r_x = 0.075;
B_l = 9.4;
B_x = 12;
sFrictionX=20;
cFrictionX=18;
sFrictionL=50;
cFrictionL=17.4;
InitWireLength=1;


if(1)
    %Simulation settings
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=3;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=0;
    
    if(runNSim)
    sim3A=sim("CraneModel.slx");
    end

    subplot(2,6,1)
    plot(simO3A.tout,simO3A.Theta,sim3A.tout,sim3A.Theta,test3A.time+1,test3A.theta-0.012)
    
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("3 A 2 sek")
    grid off

    subplot(2,6,7)
    plot(simO3A.tout,simO3A.X,sim3A.tout,sim3A.X, test3A.time+1,test3A.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid off

    disp('Sim 1 done')
end

if(1)
    %Simulation settings
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=5;

    if(runNSim)
    sim5A=sim("CraneModel.slx");
    end

    subplot(2,6,2)
    plot(simO5A.tout,simO5A.Theta,sim5A.tout,sim5A.Theta,test5A.time+1,test5A.theta)
    
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("5 A 2 sek")
    grid off

    subplot(2,6,8)
    plot(simO5A.tout,simO5A.X,sim5A.tout,sim5A.X, test5A.time+1,test5A.xTrolley-0.2)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid off

    disp('Sim 2 done')
end

if(1)
    %Simulation settings
    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=7;

    if(runNSim)
    sim7A=sim("CraneModel.slx");
    end

    subplot(2,6,3)
    plot(simO7A.tout,simO7A.Theta,sim7A.tout,sim7A.Theta,test7A.time+0.7,test7A.theta)
    
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("7 A 1 sek")
    grid off

    subplot(2,6,9)
    plot(simO7A.tout,simO7A.X ,sim7A.tout,sim7A.X, test7A.time+0.7,test7A.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid off

    disp('Sim 3 done')
end

if(1)
    %Simulation settings
    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=9;

    if(runNSim)
    sim9A=sim("CraneModel.slx");
    end

    subplot(2,6,4)
    plot(simO9A.tout,simO9A.Theta, sim9A.tout,sim9A.Theta, test9A.time+1,test9A.theta)
    
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("9 A 1 sek")
    grid off

    subplot(2,6,10)
    plot(simO9A.tout,simO9A.X, sim9A.tout,sim9A.X, test9A.time+1,test9A.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid off

    disp('Sim 4 done')
end


if(1)
    m_l = 0.95;

    %Simulation settings
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=5;
    if(runNSim)
    sim5AWC=sim("CraneModel.slx");
    end

    subplot(2,6,5)
    plot(simO5AWC.tout,simO5AWC.Theta, sim5AWC.tout,sim5AWC.Theta, test5AWC.time+1,test5AWC.theta)
    
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("9 A 1 sek Without container")
    grid off

    subplot(2,6,11)
    plot(simO5AWC.tout,simO5AWC.X, sim5AWC.tout,sim5AWC.X, test5AWC.time+1, test5AWC.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid off

    disp('Sim 5 done')
end

if(1)
    m_l = 3.89;
    InitWireLength=0.5;

    %Simulation settings
    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=7;

    if(runNSim)
    sim7A0k5=sim("CraneModel.slx");
    end

    subplot(2,6,6)
    plot(simO7A0k5.tout,simO7A0k5.Theta, sim7A0k5.tout,sim7A0k5.Theta, test7A0k5.time+1,test7A0k5.theta)
    
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("7 A 1 sek 0.5 wire")
    grid off

    subplot(2,6,12)
    plot(simO7A0k5.tout,simO7A0k5.X, sim7A0k5.tout,sim7A0k5.X, test7A0k5.time+1, test7A0k5.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid off

    disp('Sim 6 done')
end

%%
disp("Job done")


if(exportPlots)
    x0=0;
    y0=0;   
    plotwidth=250;
    height=400;
    
    figure(2)
    reduceData3Plot(simO3A.tout,simO3A.Theta,sim3A.tout,sim3A.Theta,test3A.time+1,test3A.theta-0.012)
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("3 A 2 s w. con. 1 m")
    grid off

    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X1theta.pdf','ContentType','vector')

    figure(3)
    reduceData3Plot(simO3A.tout,simO3A.X,sim3A.tout,sim3A.X, test3A.time+1,test3A.xTrolley)
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    grid on

    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X1trolley.pdf','ContentType','vector')



    figure(4)
    reduceData3Plot(simO5A.tout,simO5A.Theta,sim5A.tout,sim5A.Theta,test5A.time+1,test5A.theta)
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("5 A 2 s w. con. 1 m")
    grid off

    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X2theta.pdf','ContentType','vector')

    figure(5)
    reduceData3Plot(simO5A.tout,simO5A.X,sim5A.tout,sim5A.X, test5A.time+1,test5A.xTrolley-0.2)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid on

    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X2trolley.pdf','ContentType','vector')


    figure(6)
    reduceData3Plot(simO7A.tout,simO7A.Theta,sim7A.tout,sim7A.Theta,test7A.time+0.7,test7A.theta)
    title("7 A 1 s w. con. 1 m")
    ylabel('theta [rad]')
    grid off
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X3theta.pdf','ContentType','vector')


    figure(7)
    reduceData3Plot(simO7A.tout,simO7A.X ,sim7A.tout,sim7A.X, test7A.time+0.7,test7A.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid on

    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X3trolley.pdf','ContentType','vector')

    figure(8)
    reduceData3Plot(simO9A.tout,simO9A.Theta, sim9A.tout,sim9A.Theta, test9A.time+1,test9A.theta)
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("9 A 1 s w. con. 1 m")
    grid off
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X4theta.pdf','ContentType','vector')


    figure(9)
    reduceData3Plot(simO9A.tout,simO9A.X, sim9A.tout,sim9A.X, test9A.time+1,test9A.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid on
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X4trolley.pdf','ContentType','vector')


    figure(10)
    reduceData3Plot(simO5AWC.tout,simO5AWC.Theta, sim5AWC.tout,sim5AWC.Theta, test5AWC.time+1,test5AWC.theta)
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("5 A 1 s 1 m")
    grid off
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X5theta.pdf','ContentType','vector')


    figure(11)
    reduceData3Plot(simO5AWC.tout,simO5AWC.X, sim5AWC.tout,sim5AWC.X, test5AWC.time+1, test5AWC.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid on
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X5trolley.pdf','ContentType','vector')


    figure(12)
    reduceData3Plot(simO7A0k5.tout,simO7A0k5.Theta, sim7A0k5.tout,sim7A0k5.Theta, test7A0k5.time+1,test7A0k5.theta)
    xlabel('time [s]')
    ylabel('theta [rad]')
    title("7 A 1 s w. con. 0.5 m")
    grid off
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X6theta.pdf','ContentType','vector')


    figure(13)
    reduceData3Plot(simO7A0k5.tout,simO7A0k5.X, sim7A0k5.tout,sim7A0k5.X, test7A0k5.time+1, test7A0k5.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid on
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X6trolley.pdf','ContentType','vector')


    



    %For main report
    figure(14)
    reduceData3Plot(simO5A.tout,simO5A.Theta,sim5A.tout,sim5A.Theta,test5A.time+1,test5A.theta)
    xlabel('time [s]')
    ylabel('theta [rad]')
    grid off

    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X2thetaMain.pdf','ContentType','vector')

    figure(15)
    reduceData3Plot(simO5A.tout,simO5A.X,sim5A.tout,sim5A.X, test5A.time+1,test5A.xTrolley-0.2)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid on

    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X2trolleyMain.pdf','ContentType','vector')

     figure(16)
    reduceData3Plot(simO5AWC.tout,simO5AWC.Theta, sim5AWC.tout,sim5AWC.Theta, test5AWC.time+1,test5AWC.theta)
    xlabel('time [s]')
    ylabel('theta [rad]')
    grid off
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X5thetaMain.pdf','ContentType','vector')


    figure(17)
    reduceData3Plot(simO5AWC.tout,simO5AWC.X, sim5AWC.tout,sim5AWC.X, test5AWC.time+1, test5AWC.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid on
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X5trolleyMain.pdf','ContentType','vector')


    figure(18)
    reduceData3Plot(simO7A0k5.tout,simO7A0k5.Theta, sim7A0k5.tout,sim7A0k5.Theta, test7A0k5.time+1,test7A0k5.theta)
    xlabel('time [s]')
    ylabel('theta [rad]')
    grid off
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X6thetaMain.pdf','ContentType','vector')


    figure(19)
    reduceData3Plot(simO7A0k5.tout,simO7A0k5.X, sim7A0k5.tout,sim7A0k5.X, test7A0k5.time+1, test7A0k5.xTrolley)
    legend('Original parameters','Adjusted parameters','Measurement','Location','southoutside')
    xlabel('time [s]')
    ylabel('x-position Trolley [m]')
    grid on
    set(gcf,'position',[x0,y0,plotwidth,height])
    exportgraphics(gcf,'X6trolleyMain.pdf','ContentType','vector')

end




