% This function performs one iteration of H infinity filter localization and follows the
% algorithm in Page 353 in Optimal State Estimation by Dan Simon
% Inputs:
%           mu(t-1)                   3X1
%           sigma(t-1)                3X3
%           u                         1X1
%           z                         1Xn
%           hinff_param               struct containing the tuning parameters
% Outputs:
%           mu(t)                     3X1
%           sigma(t)                  3X3
%           flag                      true/false depending on whether a solution exists
function [mu, sigma, flag] = hinff(mu_old, sigma_old, u, z, hinff_param)

    % State variable dimension
    n = length(mu_old);
    flag = false;
    S_bar = hinff_param.S;
    
%     % Check if H_inf filter has a solution
%     if ~all(eig(inv(sigma_old) - hinff_param.theta*S_bar + hinff_param.C'*hinff_param.C./hinff_param.Q) > 0)
%         flag = true;
%         disp('No solution found for the Hinf filter problem!')
%     end

%     % Prediction and update step toethter
%     K = sigma_old*inv(eye(3)-hinff_param.theta*S_bar*sigma_old + hinff_param.C'*hinff_param.C*sigma_old./hinff_param.Q)*hinff_param.C'./hinff_param.Q; % Kalman Gain
%     nu = mod(z- hinff_param.C*mu_old + pi,2*pi)- pi; % Restrict angle between -pi and pi
%     mu = hinff_param.A*mu_old + hinff_param.B*u + hinff_param.A*K*nu;
%     mu(2,:) = mod(mu(2,:)+pi,2*pi)- pi; % Restrict angle between -pi and pi
%     sigma = hinff_param.A*sigma_old*(eye(n) -hinff_param.theta*S_bar*sigma_old + hinff_param.C'*hinff_param.C*sigma_old./hinff_param.Q)*hinff_param.A' + hinff_param.R;
%     
    % Prediction and update step toethter
    K = hinff_param.P*inv(eye(3)-hinff_param.theta*S_bar*hinff_param.P + hinff_param.C'*hinff_param.C*hinff_param.P./hinff_param.Q)*hinff_param.C'./hinff_param.Q; % Kalman Gain
    nu = mod(z- hinff_param.C*mu_old + pi,2*pi)- pi; % Restrict angle between -pi and pi
    mu = hinff_param.A*mu_old + hinff_param.B*u + hinff_param.A*K*nu;
    mu(2,:) = mod(mu(2,:)+pi,2*pi)- pi; % Restrict angle between -pi and pi
    sigma = hinff_param.P;

end