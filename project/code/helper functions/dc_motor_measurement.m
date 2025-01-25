%% Function implementing the measurement equation of the DC-Motor 
% Inputs 
% x     : State at time k                   nxN
% Outputs
% z     : Measurement at time k             1xN
function z = dc_motor_measurement(x)
    
    % Obtain the measurements 
    z = x(2,:);

end