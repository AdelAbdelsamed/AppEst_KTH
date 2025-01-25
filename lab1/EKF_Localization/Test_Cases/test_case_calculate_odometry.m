function valid =test_case_calculate_odometry()

% load test data
load test_case_odo.mat;

% store errors
errs = ones(1, NUM_TEST);

% iterate over test data
for i = 1 : NUM_TEST
    try
       tu = calculate_odometry(E_R(i), E_L(i), E_T, B, R_L, R_R, delta_t, MU(:, i));
    catch exception
        break;
    end
    
    % store euclidean norm of difference vector between ground truth
    % control signal u and computed signal tu
    errs(i) = norm(tu - U(:,i));
end

% function valid if MSE over test data below threshold
mse_err = mean(errs .^2);
THRESH_VALID = 1e-20;
valid = mse_err < THRESH_VALID;
   
end