% This function performs multinomial re-sampling
% Inputs:   
%           S_bar(t):       4XM
% Outputs:
%           S(t):           4XM
function S = multinomial_resample(S_bar)

    global M % number of particles
    

    % YOUR IMPLEMENTATION (Vectorized implementation different than one in warm up)
    % Define particles et after resampling
    % Implementation 1 
%     S = zeros(4,M);
%     S(4,:) = ones(1,M)*(1/M);
%     % Compute the CDF
%     CDF = cumsum(S_bar(4,:))';
%     % Generate M random numbers
%     r = rand(M, 1);
%     r_rep = repmat(r,1,M)';
%     % i = argmin_j CDF(j) >= r_m
%     [~, min_j] = max(r_rep <= CDF, [], 1);
%     S(1:3,:) = S_bar(1:3,min_j);
    
    % Implementation 2 
%     S = zeros(4,M);
%     cdf = cumsum(S_bar(4,:));
%     for m = 1 : M
%         r_m = rand;
%         i = find(cdf >= r_m,1,'first');
%         S(1:3,m) = S_bar(1:3,i);
%     end
%     S(4,:) = 1/M*ones(size(S_bar(4,:)));


    % Implementation 3
    S = zeros(4, M);    
    % CDF
    cdf = cumsum(S_bar(4, :));
    % Generate M random numbers between 0 and 1
    r = rand(1, M);
    [~, indices] = histc(r, [0, cdf]);
    % Resample the particles
    S(1:3, :) = S_bar(1:3, indices);
    % Assign weights
    S(4, :) = 1 / M;

    
end

