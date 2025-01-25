% This function performs one iteration of the particle filter code is
% adapted from the lab
% Inputs:
%           S(t-1)                    4XM
%           u                         1X1
%           z                         1X1
%           g                         function handle for state transition model
%           h                         function handle for measurement model
%           pf_param                  struct
%           model_param               struct
% Outputs:
%           S(t)                      4XM
function [S, flag] = pf(S, u, z, g, h, pf_param, model_param) 

flag = false;
%%% 1. Predict step including particle diffusion
noise_std = sqrt([pf_param.R(1,1); pf_param.R(2,2); pf_param.R(3,3)]);
% Compute the new particles
S_bar =  g(u, S(1:3,:), model_param) + normrnd(0, repmat(noise_std, 1, pf_param.M));
% Wrap theta around -pi to pi
S_bar(2,:) = mod(S_bar(2,:) + pi, 2*pi) - pi;
% Add weights back to the particle set
S_bar = [S_bar; S(4,:)];

%%% 2. Weight particles step
% Compute all expected observations
z_bar = h(S_bar(1:3,:));
% Compute the innovation
nu = repmat(z, 1, pf_param.M) - z_bar;
% Wrap innovation between -pi and pi
nu = mod(nu+pi,2*pi) - pi;
% Compute Likelihood
w_unnormalized = 1/(2*pi*sqrt(pf_param.Q)).*exp(-0.5*(nu.*nu)/pf_param.Q);
S_bar(4,:) = w_unnormalized./sum(w_unnormalized);

if sum(w_unnormalized) < 1e-30
    disp('Weights close to zero, Filter has diverged!')
    flag = true;
    return;
end

%%% 3. Resample particles step
switch pf_param.resample_mode
    case 0
        S = S_bar;
    case 1
            % CDF
            cdf = cumsum(S_bar(4, :));
            % Generate M random numbers between 0 and 1
            r = rand(1, pf_param.M);
            [~, indices] = histc(r, [0, cdf]);
            % Resample the particles
            S(1:3, :) = S_bar(1:3, indices);
            % Assign weights
            S(4, :) = 1 / pf_param.M;
    case 2
            % CDF
            cdf = cumsum(S_bar(4,:));
            % Generate M equally spaced random numbers in the range [0, 1/M]
            r_0 = rand / pf_param.M;
            r = r_0 + (0:(pf_param.M-1))' / pf_param.M;
            % Use the CDF
            [~, indices] = histc(r, [0, cdf]);
            % Resample the particles
            S(1:3, :) = S_bar(1:3, indices);
            % Assign equal weights to the resampled particles
            S(4, :) = 1 / pf_param.M;
end

end