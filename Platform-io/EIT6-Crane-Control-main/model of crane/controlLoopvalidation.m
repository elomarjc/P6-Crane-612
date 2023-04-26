clear
close all
clc

s = tf('s');

g = 9.82;
l_P = 1;
M_L = 2.941;
M_T = .792;
M_H = .951;
M_TL = M_L+M_H;
I_x = 0.2;
r_x = .075;
M_1 = M_T+M_L+M_H+I_x/r_x^2;
B_x = 13.4;
N_x = 48/12*72/14;
k_e = 0.609e-3;

G_x = s*(s^2+g/l_P)/(s^3*(M_1-M_TL)+s^2*B_x+s*M_1*g/l_P +B_x*g/l_P)
G_x_poles = pole(G_x)
G_x_zeroes = zero(G_x)
G_x_Gain = dcgain(G_x)

G_theta = l_P^-1*(s^2)/(s^2+g/l_P)
G_theta_zeros = zero(G_theta)
G_theta_poles = pole(G_theta)
G_theta_Gain = dcgain(G_theta)

InnerLoop = G_x/(1+1/s^2*G_theta*G_x)
pole(InnerLoop)
rlocus(InnerLoop)
figure(2)
step(InnerLoop)
figure(3)
outerLoop = InnerLoop*1/(s^2)/(1+InnerLoop*1/(s^2))
pole(outerLoop)
rlocus(outerLoop)
step(outerLoop)
margin(InnerLoop*1/(s^2))