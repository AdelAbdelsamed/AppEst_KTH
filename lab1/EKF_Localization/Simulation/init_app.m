% Initialize the app upon selection of a dataset. Draw landmarks |
% Initialize parameters | Set Visualization mode
function init_app(app, root, mapfile, dataset_id)
    %% initialize and draw map
    map_data = load([root mapfile]);
    global map
    global landmark_ids
    map = map_data(:, 2:3)'; % map including the coordinates of all landmarks | shape 2Xn for n landmarks
    landmark_ids = map_data(:, 1)'; % contains the ID of each landmark | shape 1Xn for n landmarks
    
    % include margin around landmarks
    margin = 5;
    xmin = min(map(1, :)) - margin;
    xmax = max(map(1, :)) + margin;
    ymin = min(map(2, :)) - margin;
    ymax = max(map(2, :)) + margin;

    % draw map
    cla(app.SimulationAxis)
    plot(app.SimulationAxis, map(1, :), map(2, :), 'ko')
    hold (app.SimulationAxis, 'on');
    axis(app.SimulationAxis, [xmin xmax ymin ymax])

    %% initialize parameters
    global t % global simulation time
    t = 0;
    
    global R % covariance matrix of the motion model
    global Q % covariance matrix of the measurement model
    global delta_m % percentage of true measurements retained
    global lambda_m % threshold on mahalanobis distance for outlier detection
    
    % default parameters for chosen dataset for reset button in app
    global default_R 
    global default_Q 
    global default_lambda_m
    global default_delta_m
    
    % Dataset 1
    if dataset_id == 1
        default_R = [0.01^2, 0, 0; 0, 0.01^2, 0; 0, 0, 0.01^2];
        default_Q = [0.01^2, 0; 0, 0.01^2];
        default_delta_m = 0.99; 
        default_lambda_m = chi2inv(default_delta_m, 2);
    % Dataset 2
    elseif dataset_id == 2
        default_R = [0.05^2, 0, 0; 0, 0.05^2, 0; 0, 0, 0.05^2];
        default_Q = [0.2^2, 0; 0, 0.2^2];
        default_delta_m = 0.80; 
        default_lambda_m = chi2inv(default_delta_m, 2);
    % Dataset 3
    elseif dataset_id == 3
        default_R = [1^2, 0, 0; 0, 1^2, 0; 0, 0, 1^2];
        default_Q = [0.1^2, 0; 0, 0.1^2];
        default_delta_m = 1;
        default_lambda_m = chi2inv(default_delta_m, 2);
    else
        disp('Dataset does not exist!')
    end
    
    % set global parameters to default values
    R = default_R;
    Q = default_Q;
    delta_m = default_delta_m;
    lambda_m = default_lambda_m;
    
    % enable default button
    app.DefaultButton.Enable = 'on';

    % enable spinners
    app.Spinner_R12.Enable = 'on';
    app.Spinner_R3.Enable = 'on';
    app.Spinner_Q1.Enable = 'on';
    app.Spinner_Q2.Enable = 'on';
    
    % display values in spinners
    app.Spinner_R12.Value = sqrt(R(1, 1));
    app.Spinner_R3.Value = round(sqrt(R(3, 3)) / (pi) * 180);
    app.Spinner_Q1.Value = sqrt(Q(1, 1));
    app.Spinner_Q2.Value = round(sqrt(Q(2, 2)) / (pi) * 180);

    % enable outlier detection switch
    app.OutlierSwitch.Enable = 'on';
    
    % set outlier detection switch
    if delta_m < 1
        % outlier detection enabled
        app.OutlierSwitch.Value = 'On';
        app.OutlierSpinner.Visible = 'on';
        app.OutlierSpinner.Value = round((1 - delta_m) * 100);
    else
        % outlier detection disabled
        app.OutlierSwitch.Value = 'Off';
        app.OutlierSpinner.Value = round((1 - delta_m) * 100);
    end
    
    %% initialize simulation mode
    global DATA_ASSOCIATION % use ground-truth data instead of ML data association
    global BATCH_UPDATE % perform batch update instead of sequential update
    
    % set default values
    DATA_ASSOCIATION = 'On'; 
    BATCH_UPDATE = 'On';

    % enable switches
    app.BatchUpdateSwitch.Enable = 'on';
    app.DataAssociationSwitch.Enable = 'on';
    
    % update switches
    app.BatchUpdateSwitch.Value = BATCH_UPDATE;
    app.DataAssociationSwitch.Value = DATA_ASSOCIATION;

    %% visualization mode

    global show_measurements % display a laser beam for each measurement
    global show_ground_truth % display groud truth position
    global show_odometry % display position according to odometry information
    
    % set default values
    show_measurements = true;
    show_ground_truth = true;
    show_odometry = true;
    
    % update check boxes
    app.Measurement_CheckBox.Enable = 'on';
    app.GroundTruth_CheckBox.Enable = 'on';
    app.Odometry_CheckBox.Enable = 'on';
    app.Measurement_CheckBox.Value = show_measurements;
    app.GroundTruth_CheckBox.Value = show_ground_truth;
    app.Odometry_CheckBox.Value = show_odometry;
end