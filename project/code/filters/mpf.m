% This function performs one iteration of the marginalized particle filter.
% Implemented from the paper Marginalized Particle Filters for Mixed
% Linear/Nonlinear State-Space Models by Schön et al. and optimized for the
% system at hand
% Inputs:
%           S(t-1)                    4XM
%           u                         1X1
%           z                         1X1
%           rbpf_param                  struct
%           model_param               struct
% Outputs:
%           S(t)                      4XM
function [S, flag] = mpf(S, u, z, rbpf_param, model_param) 

flag = false;
% State estimate 
S.x_est = zeros(3,1);

%%% 1. Measurement update
% Compute all expected observations
z_bar = S.xlp(2,:);
% Compute the innovation
nu = mod(repmat(z, 1, rbpf_param.M) - z_bar + pi, 2*pi) - pi;
% Compute Likelihood
M = squeeze(S.Pp(2,2,:))' + rbpf_param.Q;
% Compute w_unnormalized in a vectorized manner
w_unnormalized = exp(-0.5 * (nu.^2 ./ M));
% Normalize the weights
S.W = w_unnormalized./sum(w_unnormalized);

% Get estimate step
S.x_est(3,1) = sum(S.W.*S.xnp, 2);

if sum(w_unnormalized) < 1e-20
    disp('Weights close to zero, Filter has diverged!')
    flag = true;
    return;
end

%%% 2. Resampling step for nonlinear subsystem
cdf = cumsum(S.W);
switch rbpf_param.resample_mode
    case 1
        r = rand(1, rbpf_param.M);
    case 2
        r_0 = rand / rbpf_param.M;
        r = r_0 + (0:(rbpf_param.M-1))' / rbpf_param.M;
end
[~, indices] = histc(r, [0, cdf]);
S.xnp = S.xnp(:, indices);
S.xlp = S.xlp(:, indices);
S.Pp = S.Pp(:, :, indices);
S.xlf = S.xlf(:, indices);
S.Pf = S.Pf(:, :, indices);

% Particle filter time update and Kalman Filter
% Kalman Filter step optimized code
M = squeeze(S.Pp(2,2,:))' + rbpf_param.Q;                       % Eq. (22c)
K = squeeze(S.Pp(:,2,:))./M;                                    % Eq. (22d)
zhat = S.xlp(2,:);
S.xlf = S.xlp + K.*(mod(z - zhat + pi, 2*pi) -pi);              % Eq. (22a)
S.xlf(2,:) = mod(S.xlf(2,:) + pi, 2*pi) -pi; 
S.Pf = S.Pp - pagemtimes(reshape(K .* M, 2, 1, rbpf_param.M), 'none', reshape(K, 1, 2, rbpf_param.M), 'none'); % Eq. (22b)             
% for i = 1:rbpf_param.M
% %   M = rbpf_param.C*S.Pp(:,:,i)*rbpf_param.C' + rbpf_param.Q;    % Eq. (22c)
% %   K = S.Pp(:,:,i)*rbpf_param.C'/M;                         % Eq. (22d)
% %   zhat = rbpf_param.C*S.xlp(:,i);
% %   S.xlf(:,i)  = S.xlp(:,i) + K*(z - zhat);                      % Eq. (22a)
%     S.Pf(:,:,i) = S.Pp(:,:,i) - K(:,i)*M(i)*K(:,i)';                           % Eq. (22b)
% %   S.Pf(:,:,i) = S.Pp(:,:,i) - K*M*K';                           % Eq. (22b)
% end

S.x_est(1:2,1) = mean(S.xlf,2);    % Compute estimate for the linear states

% (4b) PF prediction according to Eq. (25b) optimized code
S.xnf = S.xnp;
% for i = 1:rbpf_param.M
%   S.xnp(i) = fn(S.xnf(i), model_param) ...
%       + S.xlf(1,i)*model_param.k_t*model_param.Ts/model_param.J_es...
%       + sqrt(rbpf_param.An*S.Pf(:,:,i)*rbpf_param.An' + rbpf_param.R(3,3))*randn(1);
% end
S.xnp = fn(S.xnf, model_param) ...
      + S.xlf(1,:).*(model_param.k_t*model_param.Ts/model_param.J_es) ...
      + sqrt(rbpf_param.An(1,1)^2*squeeze(S.Pf(1,1,:))' + rbpf_param.R(3,3)).*randn(1, rbpf_param.M);


% (4c) KF: Prediciton (Optimized code)
tic
N  = rbpf_param.An(1,1)^2.*squeeze(S.Pf(1,1,:))' + rbpf_param.R(3,3);                      % Eq. (23c)
L = [rbpf_param.Al(1,1);rbpf_param.Al(2,2)].*squeeze(S.Pf(:,1,:)).*rbpf_param.An(1,1)./N;  % Eq. (23d)
zz = S.xnp - fn(S.xnf, model_param);                                                       % Eq. (24a)
S.xlp  = [rbpf_param.Al(1,1);rbpf_param.Al(2,2)].*S.xlf + fl(S.xnf, model_param) ...
              + rbpf_param.B*u + L.*(zz - rbpf_param.An(1,1).*S.xlf(1,:));                 % Eq. (23a)
S.xlp(2,:) = mod(S.xlp(2,:) + pi,2*pi) - pi;

AlPfAlT = S.Pf;
AlPfAlT(1,1,:) = rbpf_param.Al(1,1)^2 * S.Pf(1,1,:);
AlPfAlT(1,2,:) = rbpf_param.Al(1,1) * rbpf_param.Al(2,2) * S.Pf(1,2,:);
AlPfAlT(2,1,:) = rbpf_param.Al(1,1) * rbpf_param.Al(2,2) * S.Pf(2,1,:);
AlPfAlT(2,2,:) = rbpf_param.Al(2,2)^2 * S.Pf(2,2,:);
LNLT = pagemtimes(reshape(L.*N, 2, 1, rbpf_param.M), 'none', reshape(L, 1, 2, rbpf_param.M), 'none');
S.Pp = AlPfAlT + rbpf_param.R(1:2,1:2) - LNLT;                                             % Eq. (23b)


%for i = 1:rbpf_param.M
%   N         = rbpf_param.An*S.Pf(:,:,i)*rbpf_param.An' + rbpf_param.R(3,3);              % Eq. (23c)
%   L         = rbpf_param.Al*S.Pf(:,:,i)*rbpf_param.An'*inv(N);                           % Eq. (23d)
%   zz         = S.xnp(i) - fn(S.xnf(i), model_param);                                     % Eq. (24a)
%   S.xlp(:,i)  = rbpf_param.Al*S.xlf(:,i) + fl(S.xnf(i), model_param) ...
%               + rbpf_param.B*u + L*(zz - rbpf_param.An*S.xlf(:,i)); % Eq. (23a)
%   S.Pp(:,:,i) = rbpf_param.Al*S.Pf(:,:,i)*rbpf_param.Al' + rbpf_param.R(1:2,1:2) - L*N*L'; % Eq. (23b)
%S.Pp(:,:,i) = rbpf_param.Al*S.Pf(:,:,i)*rbpf_param.Al' + rbpf_param.R(1:2,1:2) - L(:,i)*N(i)*L(:,i)'; % Eq. (23b)
%end

end


function fn_xnt = fn(xn, model_param)
    fn_xnt = (1-model_param.d_m*model_param.Ts/model_param.J_es).*xn ...
          - sign(xn).*(model_param.T_c*model_param.Ts/model_param.J_es);
end

function fl_xnt = fl(xn, model_param)
    fl_xnt = [-model_param.k_t*model_param.Ts.*xn/model_param.L;
               model_param.Ts.*xn];
end