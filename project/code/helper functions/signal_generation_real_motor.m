% Parameters
amplitudes = [3, 10, 24];        % Amplitude levels
frequencies = [10, 50];          % Frequency levels (Hz)
signalTypes = {@sin, @sawtooth, @square}; % Signal functions
signalDuration = 2;              % Duration of each signal in seconds
zeroDuration = 1;                % Duration of zero phase in seconds
samplingRate = 10000;             % Sampling frequency in Hz

% Derived parameters
dt = 1 / samplingRate;           % Time step
timeSignal = 0:dt:signalDuration-dt; % Time vector for each signal
timeZero = 0:dt:zeroDuration-dt;     % Time vector for zero phase

% Initialize signals
allSignals = [];                 % Container for all signals
signalIDs = [];                  % Container for signal IDs

% Generate signals
signalIndex = 1; % Signal ID counter
for amp = amplitudes
    for freq = frequencies
        for signalFunc = signalTypes
            % Generate current signal
            currentSignal = amp * signalFunc{1}(2 * pi * freq * timeSignal);
            
            % Append to full signal array
            allSignals = [allSignals, currentSignal, zeros(1, length(timeZero))];
            
            % Append corresponding signal IDs
            signalIDs = [signalIDs, signalIndex * ones(1, length(currentSignal)), zeros(1, length(timeZero))];
            
            % Increment signal ID
            signalIndex = signalIndex + 1;
        end
    end
end

% Create time vector for full signal
totalTime = 0:dt:(length(allSignals) - 1) * dt;

% Plot signals
figure;
subplot(2, 1, 1);
plot(totalTime, allSignals, 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Amplitude');
title('Generated Signals');
grid on;

subplot(2, 1, 2);
plot(totalTime, signalIDs, 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Signal ID');
title('Signal IDs');
grid on;
