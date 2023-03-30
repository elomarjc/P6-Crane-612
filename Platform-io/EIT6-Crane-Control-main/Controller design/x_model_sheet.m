% Constants defintion
s = tf('s');

g = 9.82;
l_P = 1;
M_T = .792;
M_H = .951;
M_C = 2.941;
M_L = M_C+M_H;
J_x = 0.13;
r_x = .075;
M_1 = M_L+M_T+J_x/r_x^2;
B_x = 12;
N_x = 48/12*72/14;
k_e = 2.96e-2;

% Current to force conversion
I2F = k_e*N_x/(r_x);

% G_theta plant ; current -> angle
G_theta = I2F*(s)/(s^3*(M_1*l_P-M_L*l_P)+s^2*(B_x*l_P) +s*(g*M_1) + g*B_x);

% G_x plant ; angle -> x_accel
G_x = (l_P*s^2 + g)/s^2;