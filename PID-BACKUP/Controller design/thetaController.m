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


figure(1)
rlocus(G_theta*H_theta)
%export_fig("thetaPlantRLocus.pdf")

D_theta = (s + .434);
figure(2)
rlocus(minreal(G_theta*H_theta*D_theta))
%export_fig("thetaFeedbackRLocus1.pdf")

figure(3)
rlocus(minreal(G_theta*H_theta*D_theta))
hold on
pole = rlocus(minreal(G_theta*H_theta*D_theta),8);
plot(real(pole), imag(pole), '^k')
hold off
legend("Root locus","Pole placement K_p=8",Location="southwest")
%export_fig("thetaFeedbackRLocus1.pdf")

clear pole
D_theta = D_theta*8;
figure(4)
pzmap(feedback(G_theta,H_theta*D_theta))
xlim([-1.5 .1])
ylim([-3.2 3.2])
pole(feedback(G_theta,H_theta*D_theta))
zero(feedback(G_theta,H_theta*D_theta))
%export_fig("thetaFBpzMapNoContainer.pdf")

%%
figure(5)
step(feedback(G_theta, H_theta*D_theta))

%%
close all
xCurrent = 1;
stepAngle(xCurrent, G_theta, H_theta, D_theta)
export_fig("innerLoopStep1ANoContainer.pdf")
xCurrent = 10;
stepAngle(xCurrent, G_theta, H_theta, D_theta)
export_fig("innerLoopStep10ANoContainer.pdf")

function stepAngle(gain, G_theta, H_theta, D_theta)
    figure
    [y, time]  = step(feedback(G_theta, H_theta*D_theta));
    
    simulation = sim("angleControllerValidation.slx");
    y = y*180/pi*gain;
    simulation.Theta1 = simulation.Theta1*180/pi;
    
    plot(time,y)
    hold on
    plot(simulation.tout, simulation.Theta1)
    hold off
    xlim([0 10])
    legend("Linear model", "Non linear model", "Location", "south east")
    grid on
    xlabel("Time [s]")
    ylabel("Angle [deg]")
    title(strcat(string(gain), " Ampere"))
end


function export_fig(name)
    x0=0;
    y0=0;
    plotwidth=650/1.5;
    plotHeight=400/1.5;
    set(gcf,'position',[x0,y0,plotwidth,plotHeight])
    
    
    exportgraphics(gcf,name,'ContentType','vector')
end
