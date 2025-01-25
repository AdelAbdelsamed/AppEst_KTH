% This function performs systematic re-sampling
% Inputs:   
%           S_bar(t):       4XM
% Outputs:
%           S(t):           4XM
function S = systematic_resample(S_bar)
	
    global M % number of particles 
    
    % YOUR IMPLEMENTATION (Vectoried Implementation)
    S = zeros(4,M);
%    % Implementation 1    
%     S(4,:) = ones(1,M)*(1/M);
%     r0_vec = zeros(M,1);
%     % Compute the CDF
%     CDF = cumsum(S_bar(4,:))';
%     % Draw a random number between 0 and 1/M
%     r0 = rand()*(1/M);
%     r0_vec(:,1) = (0:M-1)*(1/M) + r0;
%     r_rep = repmat(r0_vec,1,M)';
%     % i = argmin_j CDF(j) >= r_m
%     [~, min_j] = max(r_rep <= CDF, [], 1);
%     S(1:3,:) = S_bar(1:3,min_j);
    
%     % Implementation 2
%     cdf = cumsum(S_bar(4,:));
%     r_0 = rand / M;
%     for m = 1 : M
%         i = find(cdf >= r_0,1,'first');
%         S(1:3,m) = S_bar(1:3,i);
%         r_0 = r_0 + 1/M;
%     end
%     S(4,:) = 1/M*ones(size(S_bar(4,:)));
    
%   % Implementation 3
    % CDF
    cdf = cumsum(S_bar(4,:));
    % Generate \( M \) equally spaced random numbers in the range [0, 1/M]
    r_0 = rand / M;
    r = r_0 + (0:(M-1))' / M;
    % Use the CDF
    [~, indices] = histc(r, [0, cdf]);
    % Resample the particles
    S(1:3, :) = S_bar(1:3, indices);
    % Assign equal weights to the resampled particles
    S(4, :) = 1 / M;
    
end