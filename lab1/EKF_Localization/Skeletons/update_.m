% This function should perform the update process(sequential update).
% You need to make sure that the output sigma_bar is symmetric.
% The last line makes sure that ouput sigma_bar is always symmetric.
% Inputs:
%           mu_bar(t)       3X1
%           sigma_bar(t)    3X3
%           H_bar(t)        2X3
%           S_bar(t)        2X2
%           nu_bar(t)       2X1
% Outputs:
%           mu_bar(t)       3X1
%           sigma_bar(t)    3X3
function [mu_bar, sigma_bar] = update_(mu_bar, sigma_bar, H_bar, S_bar, nu_bar)

        % YOUR IMPLEMENTATION %

        % Compute the Kalman Gain
        K = sigma_bar*H_bar'*inv(S_bar);
        
        % Compute the updated mean
        mu_bar = mu_bar + K*nu_bar;

        % Compute the updated covariance
        sigma_bar = (eye(3) - K*H_bar)*sigma_bar;

end
