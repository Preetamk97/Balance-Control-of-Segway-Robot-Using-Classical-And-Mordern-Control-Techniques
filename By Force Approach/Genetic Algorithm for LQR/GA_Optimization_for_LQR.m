clc; clear;

%% 1) System matrices
M = 0.2; m = 1; b = 0.1; l = 0.129;
I = (1/3)*m*(l^2); g = 9.8;
p = I*(M+m)+M*m*(l^2);

A = [0  1  0  0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p  0;
     0  0  0  1;
     0 -(m*l*b)/p   m*g*l*(M+m)/p  0];

B = [0; (I+m*l^2)/p; 0; m*l/p];
C = eye(4); 
D = zeros(4,1);

assignin('base','A',A);
assignin('base','B',B);
assignin('base','C',C);
assignin('base','D',D);

%% 2) GA decision variables = [q1 q2 q3 q4 r]
nVars = 5;
lb = [1 1 1 1 0.001];
ub = [1e6 1e5 1e5 1e4 10];

opts = optimoptions('gamultiobj', ...
    'PopulationSize',30, ...
    'MaxGenerations',40, ...
    'Display','iter', ...
    'UseParallel',false, ...
    'PlotFcn',@gaplotpareto);   % live Pareto plot

%% 3) Run multiobjective GA
[xPareto,fPareto] = gamultiobj(@gaWrapperObjective, nVars, ...
                               [],[],[],[], lb, ub, [], opts);

%% 4) Build table of Pareto solutions
nSol = size(xPareto,1);
Kcells = cell(nSol,1);

for i = 1:nSol
    q = xPareto(i,1:4);
    r = xPareto(i,5);
    Q = diag(q); R = r;
    [K,~,~] = lqr(A,B,Q,R);
    Kcells{i} = K;
end

ResultTable = table( ...
    xPareto(:,1), xPareto(:,2), xPareto(:,3), xPareto(:,4), xPareto(:,5), ...
    Kcells, ...
    fPareto(:,1), fPareto(:,2), ...
    'VariableNames', {'q1','q2','q3','q4','r','K','Cost1','Cost2'});

disp('Pareto-optimal solutions and their costs:');
disp(ResultTable);

%% ===== Local functions ==========================================
function cost = gaWrapperObjective(x)
    % x = [q1 q2 q3 q4 r]
    q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); r = x(5);
    Q = diag([q1 q2 q3 q4]); 
    R = r;

    A = evalin('base','A');
    B = evalin('base','B');

    [K,~,~] = lqr(A,B,Q,R);
    assignin('base','K',K);

    cost = multiObjectiveFunction(K);   % [ITAE_pos ITAE_angle]
end
