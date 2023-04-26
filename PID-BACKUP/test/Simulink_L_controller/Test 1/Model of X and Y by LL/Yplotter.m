%Load measurements without container
Down_3 = readtable('Databehandling/3amp2sek.csv');
Down_3.y = (Down_3.y(1)-Down_3.y)+1;

Down_5 = readtable('Databehandling/5amp2sek.csv');
Down_5.y = (Down_5.y(1)-Down_5.y)+1;

Down_7 = readtable('Databehandling/Measurementsv2/7amp1sek.csv');
Down_7.y = (Down_7.y(1)-Down_7.y)+1;

Up_3 = readtable('Databehandling/-3amp2sek.csv');
Up_3.y = (Up_3.y(1)-Up_3.y)+1;

Up_5 = readtable('Databehandling/-5amp2sek.csv');
Up_5.y = (Up_5.y(1)-Up_5.y)+1;

Up_7 = readtable('Databehandling/Measurementsv2/-7amp2sek.csv');
Up_7.y = (Up_7.y(1)-Up_7.y)+1;

%Load measurements with container
Down_3_container = readtable('Databehandling/3amp2sekCon.csv');
Down_3_container.y = (Down_3_container.y(1)-Down_3_container.y)+1;

Down_5_container = readtable('Databehandling/Measurementsv2/5amp1sekcon.csv');
Down_5_container.y = (Down_5_container.y(1)-Down_5_container.y)+1;
Down_7_container = readtable('Databehandling/Measurementsv2/7amp1sekcon.csv'); %Husk at den her ikke passer!
Down_7_container.y = (Down_7_container.y(1)-Down_7_container.y)+1;

Up_4_container = readtable('Databehandling/Measurementsv2/-3amp2sekcon.csv');
Up_4_container.y = (Up_4_container.y(1)-Up_4_container.y)+1;
Up_5_container = readtable('Databehandling/Measurementsv2/-6amp2sekcon.csv');
Up_5_container.y = (Up_5_container.y(1)-Up_5_container.y)+1;
Up_7_container = readtable('Databehandling/Measurementsv2/v2-7amp2sekcon.csv');
Up_7_container.y = (Up_7_container.y(1)-Up_7_container.y)+1;


J_l = 0.14;%0.8
J_x = 0.2;
g = 9.82;
k_el = 0.71; %0.71
k_ex = 0.609;
m_t = 0.729;
r_l = 0.05;
r_x = 0.075;
B_l = 12.7; %20
B_x = 10;
sFrictionX=20;
cFrictionX=15;
sFrictionL=38; %50
cFrictionL=17.6; %90
x_min = 0;
x_max = 10;
y_min = -0.5;
y_max = 2.0;

if(1)
    m_l = 0.951;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=3;
    out1=sim("CraneModel.slx");
end
if(1)
    m_l = 3.89;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=3;
    out2=sim("CraneModel.slx");
end
if(1)
    %Simulation settings
    m_l = 0.951;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-3;
    out3=sim("CraneModel.slx");
end
if(1)
    m_l = 3.89;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-3;
    out4=sim("CraneModel.slx");
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%5 amp
if(1)
    m_l = 0.951;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=5;
    out5=sim("CraneModel.slx");
end
if(1)

    m_l = 3.89;
    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=2;
    I_LCurrent=5;
    out6=sim("CraneModel.slx");
end

if(1)
    %Simulation settings
    m_l = 0.951;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-5;
    out7=sim("CraneModel.slx");
end
if(1)
    m_l = 3.89;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-6;
    out8=sim("CraneModel.slx");
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(1)
    %Simulation settings
    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=0;
    m_l = 0.951;

    I_LStart=1;
    I_LEnd=2;
    I_LCurrent=7;
    out9=sim("CraneModel.slx");
end
if(1)

    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=0;
    m_l = 3.89;

    I_LStart=1;
    I_LEnd=2;
    I_LCurrent=7;
    out10=sim("CraneModel.slx");
end

if(1)
    %Simulation settings
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;
    m_l = 0.951;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-7;
    out11=sim("CraneModel.slx");
end
if(1)
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;
    m_l = 3.89;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-7;
    out12=sim("CraneModel.slx");
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

J_l = 0.8;%0.8
J_x = 0.2;
g = 9.82;
k_el = 0.71*1; %0.71
k_ex = 0.609;
m_t = 0.729;
r_l = 0.05;
r_x = 0.075;
B_l = 20; %20
B_x = 10;
sFrictionX=20;
cFrictionX=15;
sFrictionL=50; %50
cFrictionL=90; %90
x_min = 0;
x_max = 10;
y_min = -0.5;
y_max = 2.0;

if(1)
    m_l = 0.951;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=3;
    out=sim("CraneModel.slx");

    figure(1)

    subplot(4,3,1)
    plot(out.tout,out.L,Down_3.t-4.2,Down_3.y,out1.tout,out1.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Wire length [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("3 A 2 sek")
    grid on
end
if(1)
    m_l = 3.89;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=3;
    out=sim("CraneModel.slx");

    figure(1)

    subplot(4,3,7)
    plot(out.tout,out.L,Down_3_container.t-4.4,Down_3_container.y,out2.tout,out2.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Trolley [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("3 A 2 sek m. container")
    grid on

    %figure(2)

    %plot(Down_3.t-5.5,Down_3.y)
end
if(1)
    %Simulation settings
    m_l = 0.951;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-3;
    out=sim("CraneModel.slx");

    subplot(4,3,4)
    plot(out.tout,out.L,Up_3.t-4,Up_3.y,out3.tout,out3.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Container [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("-3 A 2 sek")
    grid on
end
if(1)
    m_l = 3.89;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-3;
    out=sim("CraneModel.slx");

    subplot(4,3,10)
    plot(out.tout,out.L, Up_4_container.t-4,Up_4_container.y,out4.tout,out4.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Trolley [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("-3 A 2 sek m. container")
    grid on
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%5 amp
if(1)
    m_l = 0.951;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=5;
    out=sim("CraneModel.slx");

    figure(1)

    subplot(4,3,2)
    plot(out.tout,out.L,Down_5.t-5,Down_5.y,out5.tout,out5.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Container [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("5 A 2 sek")
    grid on
end
if(1)

    m_l = 3.89;
    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=2;
    I_LCurrent=5;
    out=sim("CraneModel.slx");

    subplot(4,3,8)
    plot(out.tout,out.L, Down_5_container.t-5.3,Down_5_container.y,out6.tout,out6.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Trolley [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("5 A 1 sek m. container")
    grid on
end

if(1)
    %Simulation settings
    m_l = 0.951;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-5;
    out=sim("CraneModel.slx");

    subplot(4,3,5)
    plot(out.tout,out.L,Up_5.t-5.5,Up_5.y,out7.tout,out7.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Container [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("-5 A 2 sek")
    grid on
end
if(1)
    m_l = 3.89;
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-6;
    out=sim("CraneModel.slx");

    subplot(4,3,11)
    plot(out.tout,out.L, Up_5_container.t-4.5,Up_5_container.y,out8.tout,out8.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Trolley [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("-6 A 2 sek m. container")
    grid on
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(1)
    %Simulation settings
    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=0;
    m_l = 0.951;

    I_LStart=1;
    I_LEnd=2;
    I_LCurrent=7;
    out=sim("CraneModel.slx");

    figure(1)

    subplot(4,3,3)
    plot(out.tout,out.L,Down_7.t-7.3,Down_7.y,out9.tout,out9.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Container [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("7 A 1 sek")
    grid on
end
if(1)

    I_xStart=1;
    I_xEnd=2;
    I_xCurrent=0;
    m_l = 3.89;

    I_LStart=1;
    I_LEnd=2;
    I_LCurrent=7;
    out=sim("CraneModel.slx");

    subplot(4,3,9)
    plot(out.tout,out.L, Down_7_container.t-4.7,Down_7_container.y,out10.tout,out10.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Trolley [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("7 A 1 sek m. container")
    grid on
end

if(1)
    %Simulation settings
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;
    m_l = 0.951;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-7;
    out=sim("CraneModel.slx");

    subplot(4,3,6)
    plot(out.tout,out.L,Up_7.t-9.9,Up_7.y,out11.tout,out11.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Container [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("-7 A 2 sek")
    grid on

end
if(1)
    I_xStart=1;
    I_xEnd=3;
    I_xCurrent=0;
    m_l = 3.89;

    I_LStart=1;
    I_LEnd=3;
    I_LCurrent=-7;
    out=sim("CraneModel.slx");

    subplot(4,3,12)
    plot(out.tout,out.L, Up_7_container.t-5.0,Up_7_container.y,out12.tout,out12.L)
    legend('Simulation','Measurement','Location','southeast')
    xlabel('time [s]')
    ylabel('Y-position Trolley [m]')
    xlim([x_min x_max])
    ylim([y_min y_max])
    title("-7 A 2 sek m. container")
    grid on
end

disp("Job done")
