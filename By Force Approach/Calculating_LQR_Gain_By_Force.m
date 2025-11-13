clc
clear

M = 0.2;  % Mass of cart
m = 1;  % Mass of Pendulum
b = 0.1;  % Friction coefficient
l = 0.129;  % height from wheel axis to pendulum center of mass
I = (1/3)*m*(l^2);  % Moment of Inertia of pendulum 
g = -9.8; % acceleration due to gravity (m/s^2)


p = I*(M+m)+M*m*(l^2); %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 1 0 0
     0 0 1 0;
     0 0 0 1];
D = [0;
     0;
     0;
     0];

Q = [1 0 0 0;
     0 0 0 0
     0 0 1 0;
     0 0 0 0];

R = [0.1];

[K, S, E] = lqr(A, B, Q, R);




