% This function is the implementation of the measurement model.
% The bearing should be in the interval [-pi,pi)
% Inputs:
%           S(t)                           4XM
%           j                              1X1
% Outputs:  
%           z_j                              2XM
function z_j = observation_model(S, j)

    global map % map including the coordinates of all landmarks | shape 2Xn for n landmarks
    global M % number of particles

    % YOUR IMPLEMENTATION
    % Define the output
    z_j = zeros(2,M);

    % Implementation 1
    % Obtain landmark
    landmark_j = map(:,j);
    % Replicate matrices
    rep_landmark_j_x = repmat(landmark_j(1),1,M);
    rep_landmark_j_y = repmat(landmark_j(2),1,M);
    
    % Compute the observation model
    z_j(1,:) = sqrt((rep_landmark_j_x - S(1,:)).^2 + (rep_landmark_j_y - S(2,:)).^2);
    h_temp = atan2((rep_landmark_j_y - S(2,:)), (rep_landmark_j_x - S(1,:))) - S(3,:);
    
    % Ensure angle is within the range[-pi, pi]
    z_j(2,:) = mod(h_temp + pi, 2*pi) - pi;
    
    % Implementation 2 
    % Obtain landmark
%     landmark_j = map(:, j);
% 
%     % Compute the observation model directly
%     dx = landmark_j(1) - S(1, :);
%     dy = landmark_j(2) - S(2, :);
%     
%     % Compute the range and bearing
%     z_j(1, :) = sqrt(dx.^2 + dy.^2); % Range
%     z_j(2, :) = mod(atan2(dy, dx) - S(3, :) + pi, 2*pi) - pi; % Bearing

   
end
