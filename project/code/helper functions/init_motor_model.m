%% Script to initialize the Simulink model of the DC motor with the identified parameters and generate experimental data

% Define the system parameters
param.n = 5;                                      % Ratio of gearbox [-]
param.L = 11.40e-3;                               % Inductance of motor [H]
param.R = 112;                                    % Resistance of motor [Ohm]
param.k_t = 69.7e-3;                              % Motor Torque current constant [Nm/A]
param.k_emf = 7.64e-4;                            % Back emf constant [V/rad/s]
param.quant_interval = 2*pi/1000;                 % Quantization interval of the encoder (resolution = 1000/Rev)
param.T_c = 0.0009;                               % Coloumb friction [N]
param.d_v = 0.001;                                % Velocity deadband [rad/s]
param.J_es = 2.091e-5;                            % Lumped inertia [kg*m/s^2]
param.d_m = 1.28e-5;                              % Viscous friction of the motor [Nms/rad]
param.Ts = 0.0001;                                % Sample time s[]
param.std_encoder_noise = 5*param.quant_interval; % Standard deviation of encoder
param.std_current_noise = 2e-5;                   % Standard deviation of current

%% Generate experimental data
% Open motor model
mdl = 'motor_model';
open(mdl);

%frequencies = [10, 50];
%amplitudes = [3,12,24];
%wave_forms = ["sine", "sawtooth", "square"];

for amp_indx = 1:length(amplitudes)
    for freq_indx = 1:length(frequencies)
        for wave_indx = 1:length(wave_forms)
            % Select the input that you want
            input_signal.amplitude = amplitudes(amp_indx); 
            input_signal.frequency = frequencies(freq_indx);          % Note that this frequency has unit [rad/s]
            input_signal.wave_form = wave_forms(wave_indx); % Choose from 'sine', 'square', 'sawtooth'
            set_param(strcat(mdl,"/Wave Generator"), 'Amplitude', num2str(input_signal.amplitude))
            set_param(strcat(mdl,"/Wave Generator"), 'Frequency', num2str(input_signal.frequency))
            set_param(strcat(mdl,"/Wave Generator"), 'WaveForm', num2str(input_signal.wave_form))
            
            % Simulate the motor model and obtain the data for estimation
            sim_time = 2;
            simOut = sim(mdl, sim_time);
            
            % Obtain the true state vector 
            % x = [i; phi_r, omega_r]
            true_states = simOut.true_state';
            measurements = true_states(2,:);
            % Obtain input (voltage)
            controls = simOut.voltage';
            % Obtain the time vector
            time = (0:param.Ts:sim_time);

            switch frequencies(freq_indx)
                case 10
                    freq_level = "_low_freq";
                case 50 
                    freq_level = "_high_freq";
            end

            switch amplitudes(amp_indx)
                case 3
                    amp_level = "_low_amp";
                case 12 
                    amp_level = "_med_amp";
                case 24
                    amp_level = "_high_amp";
            end
            
            datafile = strcat(wave_forms(wave_indx), freq_level, amp_level);
            % Save the data
            save(datafile, "measurements", "true_states", "time", "controls")

        end
    end
end

% Plot the true state vector
figure;
subplot(4,1,1)
plot(time, controls, 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Voltage [A]')
title('Control input: Voltage')
subplot(4,1,2)
plot(time, true_states(1,:), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Current [A]')
title('State x1: Current')
subplot(4,1,3)
plot(time, true_states(2,:), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Rotor angle [rad]')
title('State x2: Rotor angle')
subplot(4,1,4)
plot(time, true_states(3,:), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Rotor velocity [rad/s]')
title('State x3: Rotor velocity')



%% Save the data
save("exp1.mat", "measurements", "true_states", "time", "controls")
