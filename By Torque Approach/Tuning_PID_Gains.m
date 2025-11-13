clc
clear

% Defining Model Constants
M_c = 1;   % Mass of chassis (kg)
M_w = 0.1;  % Mass of each wheel (kg)
L_c = 0.129; % Height of the chassis COM from the wheel centre (meter)
R_w = 0.0325; % Radius of each wheel (meter)
I_c = (1/3)*M_c*(L_c)^2; % Moment of inertia of the chassis (kg-m^2)
I_w = (1/2)*M_w*(R_w)^2; % Moment of inertia of each wheel (kg-m^2)
g = -9.81; % Gravity Constant (N.m^2/kg^2)

% Defining K constant values
k1 = (M_c*(L_c)^2)+I_c;
k2 = (M_c*(R_w)^2)+(2*M_w*(R_w)^2)+2*I_w;
k3 = M_c*R_w*L_c;
k4 = M_c*L_c*g;
k5 = (k1*k2) - (k3^2);

% Defining plant transfer function using numerator and denominator coefficients
numeratorCoeffs = [-((k1+k3)/k5)];        
denominatorCoeffs = [1 0 -((k2*k4)/k5)];
plantTF = tf(numeratorCoeffs, denominatorCoeffs);

% Print the plant transfer function
disp('Plant Transfer Function:');
plantTF          

% Tune the PID controller using pidtune
[pidControllerTuned, tuningInfo] = pidtune(plantTF, 'PID');

Kp = pidControllerTuned.Kp;
Ki = pidControllerTuned.Ki;
Kd = pidControllerTuned.Kd;

% Print the tuned PID gain values
disp('Tuned PID Controller Gains:');
disp(['Kp = ', num2str(pidControllerTuned.Kp)]);
disp(['Ki = ', num2str(pidControllerTuned.Ki)]);
disp(['Kd = ', num2str(pidControllerTuned.Kd)]);
