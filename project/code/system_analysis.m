%% State-space analysis for the system

% Define the system parameters
param.n = 5;                                      % Ratio of gearbox [-]
param.L = 11.40e-3;                               % Inductance of motor [H]
param.R = 112;                                    % Resistance of motor [Ohm]
param.k_t = 69.7e-3;                              % Motor Torque current constant [Nm/A]
param.k_emf = 7.64e-4;                            % Back emf constant [V/rad/s]
param.quant_interval = 2*pi/3600;                 % Quantization interval of the encoder (resolution = 3600/Rev)
param.T_c = 0.0009;                               % Coloumb friction [N]
param.d_v = 0.001;                                % Velocity deadband [rad/s]
param.J_es = 2.091e-5;                            % Lumped inertia [kg*m/s^2]
param.d_m = 1.28e-5;                              % Viscous friction of the motor [Nms/rad]
param.Ts = 0.0001;                                % Sample time s[]
param.std_encoder_noise = 5*param.quant_interval; % Standard deviation of encoder
param.std_current_noise = 1e-4;                   % Standard deviation of current


%% Choice of sample time
syms x;
syms Ts real;
eqn = (x - 1 + (param.R*Ts/param.L))*(x - 1 + (param.d_m*Ts/param.J_es)) - (param.k_t*Ts)^2/(param.J_es*param.L) == 0;
S = vpa(solve(eqn));
%disp('Sample Time must be chosen to satisfy the following equations')
stable_Ts1 = solve(abs(S(1)) <= 1, Ts,'ReturnConditions',1);
stable_Ts2 = solve(abs(S(2)) <= 1, Ts,'ReturnConditions',1);

%% Observability of the system
% Define the system matrices for euler representation
A_euler = [1-(param.R/param.L)*param.Ts  , 0, -param.k_t*param.Ts/param.L; 
    0                              , 1, param.Ts                  ;
    (param.k_t*param.Ts/param.J_es), 0 1-(param.d_m*param.Ts/param.J_es)];
B_euler = [param.Ts/param.L; 0; 0];
C_euler = [0, 1, 0];

% Define the system matrices for continuous-time representation
A_cont = [-(param.R/param.L), 0, -param.k_t/param.L; 
          0, 0, 1;
          (param.k_t/param.J_es), 0, -(param.d_m/param.J_es)];
B_cont = [1/param.L; 0; 0];
C_cont = [0, 1, 0];

% Check for observavility
Wo = obsv(A_cont, C_cont);
if rank(Wo) == 3
    disp('System (cont.) is observable!')
else
    disp('System (cont.) is not observable!')
end

big=expm([A_cont B_cont;zeros(1,4)]*param.Ts);
A_zoh=big(1:3,1:3);
B_zoh=big(1:3,4);
C_zoh = C_cont;
Wo = obsv(A_zoh, C_zoh);
if rank(Wo) == 3
    disp('System (zoh) is observable!')
else
    disp('System (zoh) is not observable!')
end

% Check for observavility
Wo = obsv(A_euler, C_euler);
if rank(Wo) == 3
    disp('System (Euler) is observable!')
else
    disp('System (Euler) is not observable!')
end
