%% Main script to run the estimation algorithm

% Define the system parameters
param.n = 5;                                      % Ratio of gearbox [-]
param.L = 11.40e-3;                               % Inductance of motor [H]
param.R = 112;                                    % Resistance of motor [Ohm]
param.k_t = 69.7e-3;                              % Motor Torque current constant [Nm/A]
param.k_emf = 7.64e-4;                            % Back emf constant [V/rad/s]
param.quant_interval = 2*pi/1000;                 % Quantization interval of the encoder (resolution = 3600/Rev)
param.T_c = 0.0009;                               % Coloumb friction [N]
param.d_v = 0.001;                                % Velocity deadband [rad/s]
param.J_es = 2.091e-5;                            % Lumped inertia [kg*m/s^2]
param.d_m = 1.28e-5;                              % Viscous friction of the motor [Nms/rad]
param.Ts = 0.0001;                                % Sample time s[]
param.std_encoder_noise = 5*param.quant_interval; % Standard deviation of encoder
param.std_current_noise = 1e-4;                   % Standard deviation of current

% For the model used in estimation we assumed 5% difference
param_unc = 1.0;
param.d_m = param_unc*param.d_m;
param.J_es = param_unc*param.J_es;  
param.T_c = param_unc*param.T_c;


%% Run the simulation
res = run_simulation("sine_low_amp_real_motor.mat", param, 0, true);

%% Compute the MAE across the indivdual signals

load('eval_results_real.mat');

[n_signals, n_filters] = size(error);

mae_signals = zeros(3, n_signals, n_filters);

for i = 1:n_signals
    for j = 1:n_filters
        
        if isnan(sum(cell2mat(error(i,j)), 'all'))
            continue;
        end
        mae_signals(:, i,j) = mean(abs(cell2mat(error(i,j))), 2);

    end
end
%% Plot all results
results = {};

for i = [0, 3, 5]
    
    res = run_simulation("square_low_amp_real_motor.mat", param, i, true);
    
    results = [results, res];
end
%% Plot the error results

legend_str = ["KF", "UKF","MPF"];
color_str = ['g', 'm', 'b', 'r', 'k'];
Ns = 2*3.14*10^4;

% Set figure size for IEEE column width
width = 3.39; % Single-column width in inches
height = 4.2; % Adjust the height as needed

ax = figure;
set(gcf, 'Units', 'inches', 'Position', [0, 0, width, height]);
t= (1:Ns)*1e-4;

% PLot the controls
tiledlayout(4,1)
ax1 = nexttile;
plot(t, results{1}.controls(1, 1:Ns))
grid on;
%xlabel('Time (s)')
ylabel('u (V)' ,'Interpreter','latex')

% Plot the current
ax2 = nexttile;
%subplot(4,1,2)
plot(t, results{3}.true_states(1, 1:Ns), color_str(3) ,'DisplayName', 'True State')
hold on
plot(t, results{1}.estimated_states(1, 1:Ns), color_str(1) ,'DisplayName', legend_str(1))
%plot(t, results{2}.estimated_states(1, 1:Ns), color_str(2),'DisplayName', legend_str(2))
plot(t, results{3}.estimated_states(1, 1:Ns), color_str(2) ,'DisplayName', legend_str(3))
xlim([0, 2*3.14])
grid on
%xlabel('Time (s)')
ylabel('${i_a}$ (A)', 'Interpreter','latex')
legend()

% PLot the angle
ax3 = nexttile;
%subplot(4,1,3)
% Plot the current
plot(t, results{2}.true_states(2, 1:Ns), color_str(3) ,'DisplayName', 'True State')
hold on
%plot(t, results{2}.estimated_states(2, 1:Ns), color_str(2), 'DisplayName', legend_str(2))
plot(t, results{1}.estimated_states(2, 1:Ns), color_str(1) ,'DisplayName', legend_str(1))
plot(t, results{3}.estimated_states(2, 1:Ns), color_str(2) ,'DisplayName', legend_str(3))
xlim([0, 0.2])
grid on
%xlabel('Time (s)')
ylabel('${\phi_r}$ (rad)', 'Interpreter','latex')
legend()

ax4 = nexttile;
%subplot(4,1,4)
plot(t, results{3}.estimated_states(3, 1:Ns), color_str(2) ,'DisplayName', legend_str(3))
hold on
plot(t, results{1}.estimated_states(3, 1:Ns), color_str(1) ,'DisplayName', legend_str(1))
%hold on
%plot(t, results{2}.estimated_states(3, 1:Ns), color_str(2),'DisplayName', legend_str(2))
plot(t, results{3}.true_states(3, 1:Ns), color_str(3) ,'DisplayName', 'True State')
%plot(t, results{3}.estimated_states(3, 1:Ns), color_str(2) ,'DisplayName', legend_str(3))
xlim([0, 2*3.14])
grid on
xlabel('Time (s)')
ylabel('${\omega_{r} }$ (${\frac{rad}{s}}$)', 'Interpreter','latex')
legend()

% link graphs
%linkaxes([ax1 ax2 ax3 ax4],'x')
% save plot as pdf
exportgraphics(gcf, 'exp_low_amp_square.pdf', 'ContentType', 'vector');


%% Plot the error results

legend_str = ["KF", "UKF","MPF"];
color_str = ['g', 'm', 'b', 'r', 'k'];
Ns = 2*3.14*10^4;

% Set figure size for IEEE column width
width = 3.39; % Single-column width in inches
height = 4; % Adjust the height as needed

ax = figure;
set(gcf, 'Units', 'inches', 'Position', [0, 0, width, height]);
t= (1:Ns)*1e-4;

% PLot the controls
tiledlayout(4,1)
ax1 = nexttile;
plot(t, results{1}.controls(1, 1:Ns),'m')
grid on;
%xlabel('Time (s)')
ylabel('u (V)' ,'Interpreter','latex')
n_w = 101;
n_order = 3;
smoothed_errror_1 = sgolayfilt((results{1}.true_states(1, 1:Ns) - results{1}.estimated_states(1, 1:Ns)),n_order,n_w);
smoothed_errror_2 = sgolayfilt((results{2}.true_states(1, 1:Ns) - results{2}.estimated_states(1, 1:Ns)),n_order,n_w);
smoothed_errror_3 = sgolayfilt((results{3}.true_states(1, 1:Ns) - results{3}.estimated_states(1, 1:Ns)),n_order,n_w);
% Plot the current
ax2 = nexttile;
%subplot(4,1,2)
semilogy(t, abs(smoothed_errror_1), color_str(1) ,'DisplayName', legend_str(1))
hold on
%semilogy(t, abs(smoothed_errror_2), color_str(2) ,'DisplayName', legend_str(2))
semilogy(t, abs(smoothed_errror_3), color_str(3) ,'DisplayName', legend_str(3))
xlim([0, 6.14])
grid on
%xlabel('Time (s)')
ylabel('${e_{i_a}}$ (A)', 'Interpreter','latex')
legend()

% PLot the angle
ax3 = nexttile;
%subplot(4,1,3)
smoothed_errror_1 = sgolayfilt((results{1}.true_states(2, 1:Ns) - results{1}.estimated_states(2, 1:Ns)),n_order,n_w);
smoothed_errror_2 = sgolayfilt((results{2}.true_states(2, 1:Ns) - results{2}.estimated_states(2, 1:Ns)),n_order,n_w);
smoothed_errror_3 = sgolayfilt((results{3}.true_states(2, 1:Ns) - results{3}.estimated_states(2, 1:Ns)),n_order,n_w);
% Plot the current
semilogy(t, abs(smoothed_errror_1), color_str(1) ,'DisplayName', legend_str(1))
hold on
%semilogy(t, abs(smoothed_errror_2), color_str(2) ,'DisplayName', legend_str(2))
semilogy(t, abs(smoothed_errror_3), color_str(3) ,'DisplayName', legend_str(3))
xlim([0, 0.2])
grid on
%xlabel('Time (s)')
ylabel('${e_{\phi_r}}$ (rad)', 'Interpreter','latex')
legend()

counter = 1;
ax4 = nexttile;
%subplot(4,1,4)
n_w = 151;
n_order = 3;
smoothed_errror_1 = sgolayfilt((results{1}.true_states(3, 1:Ns) - results{1}.estimated_states(3, 1:Ns)),n_order,n_w);
smoothed_errror_2 = sgolayfilt((results{2}.true_states(3, 1:Ns) - results{2}.estimated_states(3, 1:Ns)),n_order,n_w);
smoothed_errror_3 = sgolayfilt((results{3}.true_states(3, 1:Ns) - results{3}.estimated_states(3, 1:Ns)),n_order,n_w);
semilogy(t, abs(smoothed_errror_1), color_str(1) ,'DisplayName', legend_str(1))
hold on
%semilogy(t, abs(smoothed_errror_2), color_str(2) ,'DisplayName', legend_str(2))
semilogy(t, abs(smoothed_errror_3), color_str(3) ,'DisplayName', legend_str(3))
xlim([0, 6.14])
grid on
xlabel('Time (s)')
ylabel('${ e_{\omega_{r}} }$ (${\frac{rad}{s}}$)', 'Interpreter','latex')
legend()

% link graphs
%linkaxes([ax1 ax2 ax3 ax4],'x')
% save plot as pdf
exportgraphics(gcf, 'error_sim_low_amp.pdf', 'ContentType', 'vector');


%% RUN ALL DATA FILES

% Define the folder path
folderPath = 'Low_freq'; 

% Get a list of all files in the folder
dataFiles = dir(fullfile(folderPath, '*.mat')); % Adjust the extension if needed

% Store results struct in a cell array
error = cell(length(dataFiles), 6);
comp_time = cell(length(dataFiles), 6);

% Loop through each file
for i = 1:length(dataFiles)
    % Get the full file path
    filePath = fullfile(dataFiles(i).folder, dataFiles(i).name);
    
    % Run for all filters
    for j = 0:5
        result = run_simulation(filePath, param, j, false);
        error(i,j+1) = {result.error};
        comp_time(i,j+1) = {result.comp_time};
    end
    
    % Process the result as needed 
    disp(['Simulation is done for all filter types for the file: ', dataFiles(i).name]);
end

%% Evaluate data

%save('eval_results_real' , "comp_time", 'error')
mean_absolute_error = zeros(3, 6);
mean_comp_time = zeros(1, 6);
max_comp_time = zeros(1, 6);

for i = 1:size(error, 2)

    all_error = [];
    all_comp_time = [];

    for j = 1:size(error, 1)
        % Aggregate errors and computation times
        all_comp_time = [all_comp_time cell2mat(comp_time(j,i))];
        if isnan(sum(cell2mat(error(j,i)), 'all'))
            continue;
        end
        all_error = [all_error cell2mat(error(j,i))];
    end

    mean_absolute_error(:,i) = mean(abs(all_error), 2);
    mean_comp_time(:,i) = mean(all_comp_time);
    max_comp_time(:,i) = max(all_comp_time);
end