clear 
close all
clc

test = readmatrix("..//test//Controller_test_p_1.csv");
nonlinearSim = sim('plant.slx', 10);
linearSim = linearModel();

test(:,1) = (test(:,1)- test(1,1))/1000+1;
myPos = (test(:,3)-test(1,3))+1*sin(-test(:,5)*pi/180);


test(:,4) = test(:,4)-test(1,4);

figure(1)
plot(nonlinearSim.x_pos)
hold on 
plot(linearSim(:,1), linearSim(:,2))
plot(test(:,1),myPos);
hold off
legend('non linear model', 'linear model','test')
title('Model comparison')
xlabel('time [s]')
ylabel('distance [m]')
grid on 


% figure(2)
% plot(nonlinearSim.theta)
% hold on
% plot(test(:,1), test(:,5)*pi/180)
% hold off
% legend('non linear model', 'test')
% title('Model comparision, container angle')
% xlabel('time [s]')
% ylabel('angle [rad]')
% 
% figure(3)
% %stft()
% plot(nonlinearSim.tout, (nonlinearSim.thetaVEL.Data.^2).*(sin(nonlinearSim.theta.Data)))
% 
% figure(4)
% plot(nonlinearSim.tout, (nonlinearSim.thetaACC.Data).*cos(nonlinearSim.theta.Data))
% hold on
% plot(nonlinearSim.tout, nonlinearSim.thetaACC.Data)
% hold off

function toReturn = linearModel
    g = 9.82;         %gravity
    B = 9;           %dynamic friction [N]               
    mC = 2.942;       %mass of container
    mH = 0.951;       %mass of head
    m1 = 0.792;       %mass of trolley
    ke = 0.0296;      %motor constant (electric) from datasheet
    r1 = 0.16;        %radius of outer trolley gear       
    I1 = .2;      %moment of inertia of trolley gear  
    l = 1;            %length of wire                     %!!!Variable
    tg = 48/12*72/14;  %Torque gearing relationship
    

    M=mC+mH;            %Mass of head and container
    M1=M+m1+I1/r1^2;
    b1=M/M1;

    s=tf('s');
    Crane = (ke*tg*g/(r1*M1))/(s^4*(l-b1*l)+s^3*B*l/M1+s^2*g+s*B*g/M1);
    
    controller = 3;

    [x,t] = step(feedback(controller*Crane,1),10);
    t = t+1;

    toReturn = [t x];
end