function valid = test_case_associate()

% load test data
load test_case_associate.mat;

% store errors
errs_c = ones(1, NUM_TEST);
errs_outliers = ones(1, NUM_TEST);
errs_nu = ones(1, NUM_TEST);
errs_s = ones(1, NUM_TEST);
errs_h = ones(1, NUM_TEST);

% import global variables
global map
global lambda_m
global Q

% iterate over test data
for i = 1 : NUM_TEST
    
    % get data for iteration from test data
    Q = diag(QT(:,i));
    lambda_m = LAMBDA(i);
    map = M;
    z_i = Z(:,i);
    mu_bar = MU_BAR(:, i);
    sigma_bar = reshape(SIGMA_BAR(:, i), 3, 3);

    try
        [c,outlier, nu, s, h] = associate(mu_bar, sigma_bar, z_i);
    catch exception
        break;
    end
    
    % store errors for each return value of associate function
    errs_c(i) = c ~= C(i);
    errs_outliers(i) = outlier ~= OUTLIERS(i);
    errs_nu(i) = norm(nu(:) - NU(:,i));
    errs_s(i) = norm(s(:) - S(:,i));
    errs_h(i) = norm(h(:) - H(:,i));
end

% function valid if MSE over test data below threshold for each return
% value
mse_errs_c = mean(errs_c .^2);
mse_errs_outliers = mean(errs_outliers .^2);
mse_errs_nu = mean(errs_nu .^2);
mse_errs_s = mean(errs_s .^2);
mse_errs_h = mean(errs_h .^2);
THRESH_VALID = 1e-20;

valid_c = mse_errs_c < THRESH_VALID;
valid_nu = mse_errs_nu < THRESH_VALID;
valid_outliers = mse_errs_outliers < THRESH_VALID;
valid_s = mse_errs_s < THRESH_VALID;
valid_h = mse_errs_h < THRESH_VALID;

valid = valid_c && valid_nu && valid_outliers && valid_s && valid_h;

if ~valid_c
    fprintf('association computed in associate.m seems to be wrong, mse=%f\n', mse_errs_c);
end

if ~valid_outliers
    fprintf('outliers computed in associate.m seems to be wrong, mse=%f\n', mse_errs_outliers);
end

if ~valid_nu
    fprintf('nu computed in associate.m seems to be wrong, mse=%f\n', mse_errs_nu);
end

if ~valid_s
    fprintf('S computed in associate.m seems to be wrong, mse=%f\n', mse_errs_s);
end

if ~valid_h
    fprintf('H computed in associate.m seems to be wrong, mse=%f\n', mse_errs_h);
end
end
