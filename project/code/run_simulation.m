% Run Simulation of a EKF localization for landmark based maps and range
% measurements.
% Inputs:
% data_file      : Matlab datafile containing the states and the measurements
% model_param    : Model parameters of the DC Motor
% estimator_type : 0: kf, 1: hinff, 2: ekf, 3: ekf + ident, 4: ukf, 5: pf, 6: mpf 
function [result] =  run_simulation(data_file, model_param, estimator_type, draw_flag)
    
    % Load simulation data
    data = load(data_file);
    time = data.time; % Assuming time is stored as a vector
    true_states = data.true_states; % True state trajectory (3 x T)
    measurements = data.measurements; % Measurements (1 x T)
    controls = data.controls; % Control inputs (1 x T)
    
    n = size(true_states, 1); % Dimension of the state

    switch estimator_type
        case 0 % Kalman Filter
            % KF parameters
            % Continuous-time representation of linear system w\o friction
            A_cont = [-(model_param.R/model_param.L), 0, -model_param.k_t/model_param.L; 
                      0, 0, 1;
                      (model_param.k_t/model_param.J_es), 0, -(model_param.d_m/model_param.J_es)];
            B_cont = [1/model_param.L; 0; 0];
            C_cont = [0, 1, 0];
            % Obtain zoh representation
            big=expm([A_cont B_cont;zeros(1,4)]*model_param.Ts);
            kf_param.A = big(1:3,1:3);
            kf_param.B = big(1:3,4);
            kf_param.C = C_cont;
            %kf_param.R = diag([1e-6,1e-2,100]);  % Process noise covariance
            kf_param.R = diag([1e-4,1e-3,1e-2]);  % Process noise covariance
            %kf_param.Q = 0.01;                   % Measurement noise covariance
            kf_param.Q = 1e-8;

            % Initial estimates
            mu = [true_states(1,1);true_states(2,1);true_states(3,1)]; % Initial mean
            Sigma = diag([0.001,0.001,0.001]); % Initial covariance



       case 100 % Hinfinity Filter
            % Hinf parameters
            % Continuous-time representation of linear system w\o friction
            A_cont = [-(model_param.R/model_param.L), 0, -model_param.k_t/model_param.L; 
                      0, 0, 1;
                      (model_param.k_t/model_param.J_es), 0, -(model_param.d_m/model_param.J_es)];
            B_cont = [1/model_param.L; 0; 0];
            C_cont = [0, 1, 0];
            % Obtain zoh representation
            big=expm([A_cont B_cont;zeros(1,4)]*model_param.Ts);
            hinff_param.A = big(1:3,1:3);
            hinff_param.B = big(1:3,4);
            hinff_param.C = C_cont;
            hinff_param.theta = 0.5;
            hinff_param.R = diag([100,100,100]);           % Weighting of Process noise
            hinff_param.Q = 10;                          % Weighting of measurement noise
            hinff_param.S = 0.1*diag([1,1,1]);                 % Weighting of estimated state error          
            
            % Compute steady-state solution of Hinfffilter
            hinff_param.P = dare(hinff_param.A',eye(3),hinff_param.R, inv(hinff_param.C'*inv(hinff_param.Q)*hinff_param.C - hinff_param.theta*hinff_param.S));
            % Initial estimates
            mu = [controls(1,1)/model_param.R;0;0]; % Initial mean
            Sigma = diag([0.001,0.001,0.001]); % Initial covariance
        
        case 1 % Extended Kalman Filter
            % EKF parameters
            model_param.p = 100; % Determines how well the sign is approximated
            ekf_param.B = [model_param.Ts/model_param.L; 0; 0];
            ekf_param.H = [0, 1, 0];
            %ekf_param.R = diag([1e-6,1e-2,100]); % Process noise covariance
            ekf_param.R = diag([1e-4,1e-3,1e-2]); % Process noise covariance
            %ekf_param.Q = 0.01;                   % Measurement noise covariance
            ekf_param.Q = 1e-8;                   % Measurement noise covariance

            % Initial estimates
            mu = [true_states(1,1);true_states(2,1);true_states(3,1)]; % Initial mean
            Sigma = diag([0.001,0.001,0.001]); % Initial covariance

        case 2 % Extended Kalman Filter with online parameter identification
            % EKF parameters
            model_param.p = 100; % Determines how well the sign is approximated
            ekf_param.B = [model_param.Ts/model_param.L; 0; 0];
            ekf_param.H = [0, 1, 0, 0, 0, 0];
%             ekf_param.R = diag([1e-4,1e-4,1, 0, 0, 0]); % Process noise covariance
%             ekf_param.Q = 0.001;                   % Measurement noise covariance
            ekf_param.R = diag([1e-4,1e-3,1e-2, 0, 0, 0]); % Process noise covariance
            ekf_param.Q = 1e-8;   

            % Initial estimates
            mu = [true_states(1,1);true_states(2,1);true_states(3,1);model_param.d_m;model_param.J_es;model_param.T_c]; % Initial mean
            Sigma = diag([0.000001,0.000001,0.000001, 1e-6, 1e-6, 1e-6]); % Initial covariance

        case 3 % Unscented Kalman Filter
            % UKF parameters
            ukf_param.alpha = 1;
            ukf_param.beta = 0;
            ukf_param.kappa = 0;
            ukf_param.R = diag([1e-4,1e-3,1e-3]); % Process noise covariance
            ukf_param.Q = 1e-8;        % Measurement noise covariance
%             ukf_param.R = diag([1e-6,1e-5,5e-1]); % Process noise covariance
%             ukf_param.Q = 0.0001;        % Measurement noise covariance
        
            % Initial estimates
            mu = [true_states(1,1);true_states(2,1);true_states(3,1)]; % Initial mean
            Sigma = diag([0.0001,0.0001,0.0001]); % Initial covariance
        case 4 % Particle Filter
            % PF parameters
%             pf_param.Q = 0.00001; % measurement noise covariance matrix
%             pf_param.R = diag([1e-5,1e-5,1e-0]); % process noise covariance matrix
%             pf_param.M = 5000; % No.of particles
            pf_param.Q = 1e-4; % measurement noise covariance matrix
            pf_param.R = diag([1e-4,10e-0, 1e-2]); % process noise covariance matrix
            pf_param.M = 1000; % No. of particles

            pf_param.state_space_bound = [0.48;pi;260];
            pf_param.resample_mode = 2; %0=no resampling i0 + 1=vanilla resampling, 2=systematic resampling
            
            % Create initial particle set S
            S = zeros(4,pf_param.M);
            % Obtain a more accurate initial estimate
            %i0 = controls(1,1)/model_param.R;
%             S(1:3,:) = [i0 - rand(1, pf_param.M)*pf_param.state_space_bound(1) - pf_param.state_space_bound(1)/4; % sampling uniformly from the state space
%                         rand(1, pf_param.M)*2*pi - pi;
%                         2*rand(1, pf_param.M)*pf_param.state_space_bound(3) - pf_param.state_space_bound(3)];

            S(1:3,:) = [true_states(1,1) + randn(1, pf_param.M)*sqrt(0.1);
                        true_states(2,1) + randn(1, pf_param.M)*sqrt(pi/8);
                        true_states(3,1) + randn(1, pf_param.M)*sqrt(10)];
            S(4,:) =  1/pf_param.M * ones(1,pf_param.M); % initialize equal weights
        case 5 % Rao Blackwellized Particle Filter
            % RBPF parameters
            %rbpf_param.Q = 0.00001; % measurement noise covariance matrix
            %rbpf_param.R = diag([1e-5,1e-5,1e-0]);   % process noise covariance matrix
%             rbpf_param.M = 500; % No. of particles
            rbpf_param.Q = 1e-7; % measurement noise covariance matrix
            rbpf_param.R = diag([1e-5,1e-1,1e-5]);   % process noise covariance matrix
            rbpf_param.M = 500; % No. of particles
            rbpf_param.state_space_bound = 270;
            rbpf_param.resample_mode = 2; %0=no resampling 1=vanilla resampling, 2=systematic resampling

            % System matrices
            rbpf_param.B = [model_param.Ts/model_param.L; 0];
            rbpf_param.C = [0, 1];
            rbpf_param.An = [model_param.k_t*model_param.Ts/model_param.J_es, 0];
            rbpf_param.Al = [(1-model_param.R*model_param.Ts/model_param.L) 0; 0 1];
            
            % Initial mean and covariance for the conditionally linear subsystem
            xlp0 = [true_states(1,1);true_states(2,1)]; % Initial mean
            Sigmalp0 = diag([1e2,1e2]);                 % Initial covariance
            % Initial particles distribution for the nonlinear subsystem
            %S.xnp = 2*rand(1, rbpf_param.M)*rbpf_param.state_space_bound - rbpf_param.state_space_bound;
            S.xnp = true_states(3,1) + randn(1, rbpf_param.M)*sqrt(10);
            % Create initial particle set
            S.xlp = repmat(xlp0,1,rbpf_param.M);    % Conditionally linear Gaussian states
            S.Pl  = Sigmalp0;
            S.Pp  = repmat(S.Pl,[1,1,rbpf_param.M]); 
            S.W =  1/rbpf_param.M * ones(1,rbpf_param.M); % initialize equal weights
            S.xlf = zeros(size(S.xlp));                    
            S.Pf  = zeros(size(S.Pp)); 

    end

    % Preallocate storage for results
    T = length(time);
    estimated_states = zeros(n, T);
    covariance_evolution = zeros(n, T);
    error_evolution = zeros(n, T);
    comp_time = zeros(1,T);

    % Online simulation
    figure(1); clf;
    for t = 2:T
        % Run one step of the estimator
        switch estimator_type
            case 0 % Kalman Filter
                tic;
                [mu, Sigma] = kf(mu, Sigma, controls(:, t), measurements(:, t), kf_param);
                comp_time(1,t) = toc;

            case 100 % H inifity Filter
                tic;
                [mu, Sigma, flag] = hinff(mu, Sigma, controls(:, t), measurements(:, t), hinff_param);
                comp_time(1,t) = toc;
                if flag
                   break;
                end
            
            case 1 % Extended Kalman Filter
                tic;
                [mu, Sigma] = ekf(mu, Sigma, controls(:, t), measurements(:, t), ekf_param, model_param);
                comp_time(1,t) = toc;

            case 2 % Extended Kalman Filter with parameter identification 
                tic;
                [mu, Sigma] = ekf_ident(mu, Sigma, controls(:, t), measurements(:, t), ekf_param, model_param);
                comp_time(1,t) = toc;
            
            case 3 % Unscented Kalman Filter
                tic;
                [mu, Sigma] = ukf(mu, Sigma, controls(:, t), measurements(:, t), ...
                                              @dc_motor_state_transition, @dc_motor_measurement, ukf_param, model_param);
                comp_time(1,t) = toc;

            case 4 % Standard Particle Filter
                tic;
                [S, flag] = pf(S, controls(:, t), measurements(:, t), @dc_motor_state_transition, @dc_motor_measurement, pf_param, model_param);
                comp_time(1,t) = toc;
                if flag
                    break;
                end
                mu = sum(S(4,:).*S(1:3,:), 2);

            case 5 % Rao Blackwellized Particle Filter
                tic;
                [S, flag] = mpf(S, controls(:, t), measurements(:, t), rbpf_param, model_param);
                comp_time(1,t) = toc;
                if flag
                    break;
                end
                mu = S.x_est;
         end

        % Store results
        estimated_states(:, t) = mu(1:3);
        %covariance_evolution(:, t) = diag(Sigma);
        error_evolution(:, t) = true_states(:, t) - mu(1:3);


        if mod(t, 10000) == 0 && draw_flag
            subplot(4, 1, 1);
            sgtitle('Simulation: DC Motor')
            plot(time(1,1:t), controls(1, 1:t), 'b'); hold on;
            grid on;
            legend('Control')
            ylabel('Control: Voltage [V]'); 
            xlabel('Time [s]');
            subplot(4, 1, 2);
            plot(time(1,1:t), true_states(1, 1:t), 'b'); hold on;
            plot(time(1,1:t), estimated_states(1, 1:t), 'r');
            grid on;
            legend('True Current', 'Estimated Current')
            ylabel('State 1: i [A]'); 
            xlabel('Time [s]');
            subplot(4, 1, 3);
            plot(time(1,1:t), true_states(2, 1:t), 'b'); hold on;
            plot(time(1,1:t), estimated_states(2, 1:t), 'r');
            grid on;
            legend('True rotor angle', 'Estimated rotor angle')
            ylabel('State 2: phi_r [rad]'); 
            xlabel('Time [s]');
            subplot(4, 1, 4);
            plot(time(1,1:t), true_states(3, 1:t), 'b'); hold on;
            plot(time(1,1:t), estimated_states(3, 1:t), 'r');
            grid on;
            legend('True rotor speed', 'Estimated rotor speed')
            ylabel('State 3: omega_r [rad/s]'); 
            xlabel('Time [s]');
            drawnow;
        end
    end

%     % Plot covariance evolution
%     figure(2); clf;
%     subplot(n,1, 1);
%     for i = 1:n
%         subplot(n, 1, i);
%         plot(time, covariance_evolution(i, :));
%         title(['Covariance of State ', num2str(i)]);
%         xlabel('Time'); ylabel('Covariance');
%         grid on;
%     end

    % Plot error evolution
    figure(3); clf;
    for i = 1:n
        subplot(n, 1, i);
        plot(time, error_evolution(i, :));
        title(['Error of State ', num2str(i), ' MAE = ', num2str(mean(abs(error_evolution(i,:))))]);
        xlabel('Time'); ylabel('Error');
        grid on;
    end

    % Compute the normalized mean absolute error
    disp('..................................................')
    mae = mean(abs(estimated_states - true_states),2);
    disp(['MAE Current =',num2str(mae(1))])
    disp(['MAE Angle =',num2str(mae(2))])
    disp(['MAE Angle speed =',num2str(mae(3))])

    % Save the results
    result.error = error_evolution;
    result.comp_time = comp_time;
    result.estimated_states = estimated_states;
    result.true_states = true_states;
    result.controls = controls;

    
end

