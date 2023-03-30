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

D_theta = (s + 1.5)*8;

InnerLoop = feedback(G_theta, D_theta*H_theta);

G_x = -(l_P*s^2+ g)/s^2;


figure
rlocus(minreal(InnerLoop*G_x))

figure
rlocus(minreal(feedback(G_theta, 1*D_theta*H_theta)*G_x*(2+2*s+1/2/s)))
