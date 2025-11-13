clc
clear

% Defining Model Constants
M_c = 1;   % Mass of chassis
M_w = 0.1;  % Mass of each wheel
L_c = 0.129; % Height of the chassis COM from the wheel centre
R_w = 0.0325; % Radius of each wheel
I_c = (1/3)*M_c*(L_c)^2; % Moment of inertia of the chassis
I_w = (1/2)*M_w*(R_w)^2; % Moment of inertia of each wheel
g = -9.81; % Gravity Constant

% Defining K constant values
k1 = (M_c*(L_c)^2)+I_c;
k2 = (M_c*(R_w)^2)+(2*M_w*(R_w)^2)+2*I_w;
k3 = M_c*R_w*L_c;
k4 = M_c*L_c*g;
k5 = (k1*k2) - (k3^2);

% A-Matrix
A = [0  1  0  0;
    (k2*k4)/k5  0  0  0;
    0  0  0  1;
    -(k1*k4)/k5  0  0  0
    ];

% B-Matrix
B = [0;
    -(k2+k3)/k5;
    0;
    (k1+k3)/k5
    ];

% Q-Matrix  (State Cost)
Q = [1000 0  0  0;
    0  500  0  0;
    0  0  1000  0;
    0  0  0  500
    ];

% R-Matrix  (Control Cost)
R = [0.1];

% Checking controllability of the system and calculating the LQR gain if
% the system is controllable
C_o = ctrb(A,B);
r = rank(C_o);
if r == 4
    disp('The system is controllable.')
    [K S E] = lqr(A,B,Q,R);
    disp('Here is the K Gain Matrix required:')
    K
    disp('Here are the eigen values of A-BK Matrix:')
    E
else
    disp('The system is not controllable.')
end

