%{
    This matlab file utilizes MEIT class and plots its values
    and can add measured point to the frequency characteristic
        
    *   Copyright (c) 2023 Roman Vaněk
    *   All rights reserved.
    *
    *   Dept. of Radio Electronics, Brno University of Technology, Czechia
%}

% Global variables to control flow and target plot 
global meit;
global plotHandler;
global frequencyCharacteristic;

% Configuration Parameters
COM_PORT = 'COM3';
BAUD_RATE = 115200;
N_INIT_SAMPLES = 500;

% Create figure with specific ID for easy reference
figure(1);
% Clear figure to ensure it's clean on each script run
clf; 

meit = MEIT(COM_PORT, BAUD_RATE);

frequencyCharacteristic = [];

% GUI Controls
hSave = uicontrol('Style', 'pushbutton', 'String', 'Save Plot', ...
                  'Position', [0 20 80 40], 'Callback', @savePlot);
hLoad = uicontrol('Style', 'pushbutton', 'String', 'Load Plot', ...
                  'Position', [100 20 80 40], 'Callback', @loadPlot);
hLoadIndex = uicontrol('Style', 'pushbutton', 'String', 'Load index', ...
                  'Position', [200 20 80 40], 'Callback', @loadIndex);
hLoadFreq = uicontrol('Style', 'pushbutton', 'String', 'Load freq. char.', ...
                  'Position', [300 20 80 40], 'Callback', @loadFreq);

hMeasure = uicontrol('Style', 'pushbutton', 'String', 'Measure', ...
                  'Position', [500 20 80 40], 'Callback', @appendMeasurement);
hMeasureManual = uicontrol('Style', 'pushbutton', 'String', 'Measure manual freq', ...
                  'Position', [600 20 160 40], 'Callback', @appendMeasurementFreq);


hIncrease = uicontrol('Style', 'pushbutton', 'String', 'Increase Gain', ...
                  'Position', [900 20 80 40], 'Callback', @set_increase);
hDecrease = uicontrol('Style', 'pushbutton', 'String', 'Decrease Gain', ...
                  'Position', [1000 20 80 40], 'Callback', @set_decrease);
hSetGain = uicontrol('Style', 'pushbutton', 'String', 'Set Gain', ...
                  'Position', [1100 20 80 40], 'Callback', @setGain);

hRead = uicontrol('Style', 'pushbutton', 'String', 'Read', ...
                  'Position', [1300 20 80 40], 'Callback', @read);
hSetSampling = uicontrol('Style', 'pushbutton', 'String', 'Set Sampling', ...
                  'Position', [1400 20 80 40], 'Callback', @setSampling);

% Initialize plot elements
plotHandler = struct( ...
    'plot',             [], ...
    'textMAX',          [], ...
    'textRMS',          [], ...
    'textFREQ',         [], ...
    'textPHASE',        [], ...
    'textIMPEDANCE1',   [], ...
    'textIMPEDANCE2',   [], ...
    'meanLine',         [], ...
    'zeroCross',        [], ...
    'axis',             []);

% Create a new figure and set its title
figure(1);


% Init voltage plot
subplot(2,1,1);
hold on;
plotHandler(1) = setupInitialPlot('Voltage [V]', N_INIT_SAMPLES);
title('Channel 1')
% Init current plot
subplot(2,1,2);
hold on;
plotHandler(2) = setupInitialPlot('Voltage [V]', N_INIT_SAMPLES);
title('Channel 2')

% plotCharacteristics(frequencyCharacteristic);

while true
    pause(1);
end

function plotHandler = setupInitialPlot(title, N_SAMPLES)
    % Initial plot setup
    ylabel(title);
    xlabel('Sample Index [-]');

    % Horizontal line at 0 volts
    yline(0, '--r', 'LineWidth', 2); 
    
    % Set-up max and min value of axis
    axis([1 N_SAMPLES -3.3 3.3]);

    % Define handlers within plotHandler struct
    plotHandler.plot = plot(zeros(N_SAMPLES, 1), 'b.', 'LineWidth', 2);
    % Black dashed line for mean voltage
    plotHandler.meanLine = yline(0, 'k--', 'LineWidth', 1.5); 
    % Red points for displaying zero-crosses
    hold on;
    plotHandler.zeroCross = plot(nan, nan, 'rx', 'LineWidth', 2, 'MarkerSize', 10);
    % Define axis handler for dynamic scaling
    plotHandler.axis = gca;

    % Define vertical spacing between text elements
    textYSpacing = plotHandler.axis.YLim(2) / 8;
    % Define horizontal offset
    textXSpacing = plotHandler.axis.XLim(2) / 50;
    % Calculate text position based on axis limits
    textY = plotHandler.axis.YLim(2) - plotHandler(1).axis.YLim(2) / 10;

    % Text object for displaying measurements
    plotHandler.textMAX = text(textXSpacing, textY, 0, [' MAX: 0 V'], 'FontSize', 13);
    plotHandler.textRMS = text(textXSpacing, textY - textYSpacing, [' RMS: 0 V'], 'FontSize', 13);
    plotHandler.textFREQ = text(textXSpacing, textY - 2 * textYSpacing, ['0 Hz'], 'FontSize', 13);

    textYPosition = 1.15;
    % Display phase shift
    plotHandler.textPHASE = text(0.1, textYPosition, ['Phase Shift: ', '0', ' °'], 'FontSize', 13, 'Units', 'normalized');
    % Display impedance
    plotHandler.textIMPEDANCE1 = text(0.1, textYPosition - 0.05, ['Impedance: ', '0', '<', '0', ' °'], 'FontSize', 13, 'Units', 'normalized');
    plotHandler.textIMPEDANCE2 = text(0.1, textYPosition - 0.1, ['Impedance: ', '0', ' Ω'], 'FontSize', 13, 'Units', 'normalized');
end

function appendMeasurement(~, ~)
    global meit;
    global frequencyCharacteristic;

    if isfield(frequencyCharacteristic, 'frequencyCharacteristic')
        frequencyCharacteristic = frequencyCharacteristic.frequencyCharacteristic;
    end
    % Append measurements to array
    frequencyCharacteristic = [frequencyCharacteristic, meit.measuredData];
    % Plot the frequency characteristic
    plotCharacteristics(frequencyCharacteristic);
end

function appendMeasurementFreq(~, ~)
    global meit;
    global frequencyCharacteristic;
    
    if isfield(frequencyCharacteristic, 'frequencyCharacteristic')
        frequencyCharacteristic = frequencyCharacteristic.frequencyCharacteristic;
    end
    % Append measurements to array
    freq = getInput('Enter frequency of measured signal:', 1000000, 'integer');
    meit.measuredData.frequencyVoltage = freq;
    meit.measuredData.frequencyCurrent = freq;
    frequencyCharacteristic = [frequencyCharacteristic, meit.measuredData];
    % Plot the frequency characteristic
    plotCharacteristics(frequencyCharacteristic);
end

function read(~, ~)
    update([]);
end

function update(measuredData)
    global plotHandler;
    global meit;

    % if the parameter measuredData is empty read the data from the MEIT
    if isempty(measuredData)
        % Get voltage and current data along with error code
        [voltageData, currentData, errorCode] = meit.get_measurement([]);

        % Check if there was an error in getting the measurement
        if ~errorCode
            return;
        end

        % Calculate mean values
        meanVoltage = mean(voltageData);
        meanCurrent = mean(currentData);

        % Center data around mean
        voltageData = voltageData - meanVoltage;
        currentData = currentData - meanCurrent;

        voltageData = voltageData(10:end);
        currentData = currentData(10:end);
        
        % Calculate measurements
        measuredData = meit.calculate(voltageData, currentData);
    end

    measuredData = meit.calculate(measuredData.voltageData, measuredData.currentData);
    meit.measuredData = measuredData;

    if any(isnan(measuredData.voltageData(:)))
        measuredData.voltageData(:) = 0;
    end
    if any(isnan(measuredData.currentData(:)))
        measuredData.currentData(:) = 0;
    end

    % Set-up max and min value of axis according to recieved data
    maxYaxisVoltage = 1.2 * measuredData.maxVoltage;
    maxYaxisCurrent = 1.2 * measuredData.maxCurrent;

    if isempty(measuredData.voltageData)
        maxYaxisVoltage = 1;
        nSamplesVoltage = 500;
    else
        nSamplesVoltage = length(measuredData.voltageData);
    end
    if isempty(measuredData.currentData)
        maxYaxisCurrent = 1;
        nSamplesCurrent = 500;
    else
        nSamplesCurrent = length(measuredData.currentData);
    end

    % Dynamicly scale axis
    ylim(plotHandler(1).axis, [-maxYaxisVoltage, maxYaxisVoltage]);
    xlim(plotHandler(1).axis, [1, nSamplesVoltage]);
    ylim(plotHandler(2).axis, [-maxYaxisCurrent, maxYaxisCurrent]);
    xlim(plotHandler(2).axis, [1, nSamplesCurrent]);
    % Plot the data
    set(plotHandler(1).plot, 'YData', measuredData.voltageData);
    set(plotHandler(2).plot, 'YData', measuredData.currentData);


    % Define vertical spacing between text elements
    textYSpacingVoltage = plotHandler(1).axis.YLim(2) / 8;
    textYSpacingCurrent = plotHandler(2).axis.YLim(2) / 8;
    % Define horizontal offset
    textXSpacingVoltage = plotHandler(1).axis.XLim(2) / 50;
    textXSpacingCurrent = plotHandler(2).axis.XLim(2) / 50;
    % Calculate text position based on axis limits
    textYVoltage = plotHandler(1).axis.YLim(2) - plotHandler(1).axis.YLim(2) / 10;
    textYCurrent = plotHandler(2).axis.YLim(2) - plotHandler(2).axis.YLim(2) / 10;
    
    % Update text positions for Voltage plotHandler
    set(plotHandler(1).textMAX, 'Position',     [textXSpacingVoltage, textYVoltage, 0]);
    set(plotHandler(1).textRMS, 'Position',     [textXSpacingVoltage, textYVoltage - textYSpacingVoltage, 0]);
    set(plotHandler(1).textFREQ, 'Position',    [textXSpacingVoltage, textYVoltage - 2 * textYSpacingVoltage, 0]);
    
    % Update text positions for Current plotHandler
    set(plotHandler(2).textMAX, 'Position',     [textXSpacingCurrent, textYCurrent, 0]);
    set(plotHandler(2).textRMS, 'Position',     [textXSpacingCurrent, textYCurrent - textYSpacingCurrent, 0]);
    set(plotHandler(2).textFREQ, 'Position',    [textXSpacingCurrent, textYCurrent - 2 * textYSpacingCurrent, 0]);


    % Format measurement data as strings
    % Update text display with measurement data strings
    set(plotHandler(1).textMAX, 'String', sprintf('Voltage MAX: %.4f V', measuredData.maxVoltage));
    set(plotHandler(1).textRMS, 'String', sprintf('Voltage RMS: %.4f V', measuredData.rmsVoltage));
    set(plotHandler(1).textFREQ, 'String', sprintf('Frequency: %.0f Hz', measuredData.frequencyVoltage));

    set(plotHandler(2).textMAX, 'String', sprintf('Current MAX: %.4f V', measuredData.maxCurrent));
    set(plotHandler(2).textRMS, 'String', sprintf('Current RMS: %.4f V', measuredData.rmsCurrent));
    set(plotHandler(2).textFREQ, 'String', sprintf('Frequency: %.0f Hz', measuredData.frequencyCurrent));


    % Calculate magnitude and phase of impedance
    magnitude = abs(measuredData.impedance);
    phase = angle(measuredData.impedance) * 180 / pi; % Convert phase to degrees
    % Display impedance
    set(plotHandler(1).textPHASE, 'String', sprintf('Phase Shift: %.4f °', measuredData.phaseShift));
    set(plotHandler(1).textIMPEDANCE1, 'String', sprintf('Impedance: %.4f < %.4f °  Ω', magnitude, phase));
    set(plotHandler(1).textIMPEDANCE2, 'String', ['Impedance: ', num2str(measuredData.impedance), ' Ω']);

    set(plotHandler(2).textPHASE, 'Visible', false);
    set(plotHandler(2).textIMPEDANCE1, 'Visible', false);
    set(plotHandler(2).textIMPEDANCE2, 'Visible', false);


    % Calculate mean values
    meanVoltage = mean(measuredData.voltageData);
    meanCurrent = mean(measuredData.currentData);
    % Update mean line position
    plotHandler(1).meanLine.Value = meanVoltage;
    plotHandler(2).meanLine.Value = meanCurrent;

    % Plot zero crossing points
    hold on;
    set(plotHandler(1).zeroCross, 'XData', measuredData.zeroCrossVoltage, 'YData', zeros(1, length(measuredData.zeroCrossVoltage)));
    set(plotHandler(2).zeroCross, 'XData', measuredData.zeroCrossCurrent, 'YData', zeros(1, length(measuredData.zeroCrossCurrent)));
end

function setGain(~, ~)
    global meit;
    % Get gain index from user
    gain_index = getInput('Enter Gain index (0 - 42):', 0, 'integer');

    % Check if input is valid
    if ~isempty(gain_index) && gain_index >= 0 && gain_index <= 42
        meit.set_gain(gain_index);
    else
        disp('Error: Invalid gain index. Please enter a value between 0 and 42.');
    end
end

function setSamlingDivider1(~, ~)
    setSamlingDivider(0);
end
function setSamlingDivider2(~, ~)
    setSamlingDivider(1);
end
function setSamlingDivider3(~, ~)
    setSamlingDivider(2);
end
function setSamlingDivider4(~, ~)
    setSamlingDivider(3);
end
function setSampling(~, ~)
    global meit;
    % Get gain index from user
    devider = getInput('Enter sampling frequency (2 571 428.571 Hz) divider (1 - 65535):', 1, 'integer');
    devider = devider - 1;
    % Check if input is valid
    if devider >= 0 && devider <= 65534 
        setSamlingDivider(devider);
    else
        disp('Error: Invalid gain index. Please enter a value between 0 and 42.');
    end
end

function setSamlingDivider(devider)
    global meit;
    % Check if input is valid
    if devider >= 0 && devider < 65535
        meit = meit.set_fsampling(devider);
    end
end

function set_increase(~, ~)
    global meit;
    meit.set_increase();
end

function set_decrease(~, ~)
    global meit;
    meit.set_decrease();
end

function savePlot(~, ~)
    global meit;

    % Get filename from user
    fileName = getInput('Enter filename to save measurement data to:', 'measuredData.mat', 'char');

    % Check if user provided a valid filename
    if ~isempty(fileName)
        % Save the measurement data to the specified file
        measuredData = meit.measuredData;
        save(fileName, 'measuredData');
        disp(['Measurement data saved to: ' fileName]);
    else
        disp('Operation cancelled by user.');
    end
end

function loadPlot(~, ~)
    % Get filename from user
    fileName = getInput('Enter filename to load measurement data from:', 'measuredData.mat', 'char');

    % Check if the file exists
    if exist(fileName, 'file') == 2
        % Load the measurement data from the file
        loadedData = load(fileName);
        if isfield(loadedData, 'measuredData')
            % Update the measurement data in the meit object
            update(loadedData.measuredData);
            disp(['Measurement data loaded from: ' fileName]);
        else
            disp('Error: The loaded file does not contain measurement data.');
        end
    else
        disp('Error: The specified file does not exist.');
    end
end

function loadFreq(~, ~)
    global frequencyCharacteristic;
    % Get filename from user
    fileName = getInput('Enter filename to load measurement data array from frequency Char:', 'freqChar.mat', 'char');

    % Check if the file exists
    if exist(fileName, 'file') == 2
        % Load the measurement data from the file
        frequencyCharacteristic = load(fileName);
        plotCharacteristics(frequencyCharacteristic);
        % Update the measurement data in the meit object
        disp(['Measurement data loaded from: ' fileName]);
    else
        disp('Error: The specified file does not exist.');
    end
end

function loadIndex(~, ~)
    global frequencyCharacteristic;
    % Get filename from user
    index = getInput('Enter index of frequency characteristics to load:', 1, 'integer');

    if isfield(frequencyCharacteristic, 'frequencyCharacteristic')
        frequencyCharacteristic = frequencyCharacteristic.frequencyCharacteristic;
    end

    % Check if input is valid
    if index > 0 && index <= length(frequencyCharacteristic) 
        update(frequencyCharacteristic(index));
    else
        disp(['Error: Invalid frequency characteristis array index. Please enter a value between 0 and ', length(frequencyCharacteristic), '.']);
    end
end

function value = getInput(promptMsg, defaultVal, type)
    % Prompt user to enter input using inputdlg
    prompt = {promptMsg};
    dlgtitle = 'Input';
    dims = [1 35];
    % Use default value as provided
    if strcmpi(type, 'integer') || strcmpi(type, 'double')
        definput = {num2str(defaultVal)};
    else
        definput = {defaultVal};
    end

    userInput = inputdlg(prompt, dlgtitle, dims, definput);

    % Check if user provided input or canceled
    if ~isempty(userInput)
        % Extract user input from cell array
        inputVal = userInput{1};
        
        % Check if input is valid based on type
        if strcmpi(type, 'integer')
            % Check if input can be converted to an integer
            if ~isempty(str2double(inputVal))
                value = str2double(inputVal);  % Return valid input
            else
                error('Input must be an integer.');
            end
        elseif strcmpi(type, 'double')
            % Check if input can be converted to a number
            if ~isempty(str2double(inputVal))
                value = str2double(inputVal);  % Return valid input
            else
                error('Input must be a number.');
            end
        elseif strcmpi(type, 'char')
            value = inputVal;  % Return valid input
        else
            error('Invalid input type specified.');
        end
    else
        % User canceled, return empty
        value = [];
    end
end
