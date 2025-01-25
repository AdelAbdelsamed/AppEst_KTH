% This function is the implementation of the measurement model.
% The bearing should be in the interval [-pi,pi)
% Inputs:
%           x(t)                           3X1
%           j                              1X1
% Outputs:  
%           h                              2X1
function z_j = observation_model(x, j)

    global map % map | 2Xn for n landmarks

    % YOUR IMPLEMENTATION %
       
    % Obtain the landmark corresponding to j
    landmark_j = map(:,j);
    
    % Define the output
    z_j = zeros(2,1);
    
    % Compute the observation model
    z_j(1,1) = sqrt((landmark_j(1) - x(1))^2 + (landmark_j(2) - x(2))^2);
    h_temp = atan2((landmark_j(2) - x(2)), (landmark_j(1) - x(1))) - x(3);
    
    % Ensure angle is within the range[-pi, pi]
    z_j(2,1) = mod(h_temp + pi, 2*pi) - pi;

end
