%{
    This class provides a simple API for communicating with a modular system
    for electrical impedance tomography.
        
    *   Copyright (c) 2023 Roman Vaněk
    *   All rights reserved.
    *
    *   Dept. of Radio Electronics, Brno University of Technology, Czechia

    Example usage:

    % Create an instance of the meit class
    meitObj = meit;

    % Set the gain index
    meitObj.set_gain(10);

    % Read measured voltage and current buffers and do not store them to .mat file
    [voltageData, currentData, isDataReady] = get_measurement(~);
%}
classdef MEIT
    properties (Access = private, Constant)
        % Clock freqency (input of a timer used to drive ADC samling)
        FREQ double = 72e6;
    end
    properties
        % Port for communication (default: COM3)
        port char;
        % Baud rate for communication (default: 115200)
        baudRate double;
        % Sampling frequency of one channel (voltage/current channel) in Hz
        % used for phaseshift and frequency calculation
        samplingFrequency double;
        % Structure for storing measured data
        measuredData = struct(... 
            'maxVoltage',   [], 'maxCurrent',       [], 'rmsVoltage',       [], ...
            'rmsCurrent',   [], 'zeroCrossVoltage', [], 'zeroCrossCurrent', [], ...
            'phaseShift',   [], 'frequencyVoltage', [], 'frequencyCurrent', [], ...
            'voltageData',  [], 'currentData',      [], 'impedance',        []);
    end
    
    methods
        function obj = MEIT(port, baudRate)
            % Constructor function to initialize object with provided or default values
            if nargin > 0
                obj.port = port;
            end
            if nargin > 1
                obj.baudRate = baudRate;
            end
            obj. samplingFrequency = MEIT.FREQ / (14 * (0 + 1) * (1 + 1));
        end
        
        % ***************** Comunication methods ********************** %
        function read = send(obj, cmd, msg, arg1, arg2)
            read = '';
            try
                s = serialport(obj.port, obj.baudRate);
                % Configure the '\n' as a terminator
                configureTerminator(s, "LF");
                
                command = append(cmd, ' ', msg);
                if ~isempty(arg1)
                    command = append(command, ' ', num2str(arg1));
                end
                if ~isempty(arg2)
                    command = append(command, ' ', num2str(arg2));
                end
                
                writeline(s, command);
                disp('Sent MEASURE command to STM32');
                
                % Wait for a short time to allow data to be received
                % Check if data is available to read
                if strcmpi(cmd, 'GET')
                    tic
                    for i = 0:2000
                        if s.NumBytesAvailable > 0
                            break
                        end
                        pause(1 / obj.samplingFrequency);
                    end
                    if s.NumBytesAvailable > 0
                        % Read the data
                        read = readline(s);
                    else
                        disp(['Timeut occurs while reading from the device on ' obj.port '.']);
                    end
                    toc
                end
                
            catch ME
                disp(['Error communicating with the device on ' obj.port ': ' ME.message]);
            end
            
            % Close the serial port if it was successfully opened
            if exist('s', 'var')
                delete(s);
            end
        end
        
        function set_gain(obj, gain_index)
            obj.send('SET', 'GAIN', gain_index, '');
        end
        
        function set_increase(obj)
            obj.send('SET', 'INCREASE', '', '');
        end
        
        function set_decrease(obj)
            obj.send('SET', 'DECREASE', '', '');
        end
        
        function obj = set_fsampling(obj, devider)
            obj.send('SET', 'FSAMPLING', devider, '');
            obj.samplingFrequency = MEIT.FREQ / (14 * (devider + 1) * (1 + 1));
            disp(['Sampling freq: ' num2str(obj.samplingFrequency) ' Hz']);
        end
        
        function [voltageData, currentData, isDataReady] = get_measurement(obj, filename)
            % Initialize output
            isDataReady = false;
            voltageData = 0;
            currentData = 0;
            % Define separator used to separate ADC data:
            % Defaultly transmitted as "voltage,current,voltage,..."
            separator = ',';
            
            read = obj.send('GET', 'MEASUREMENT', '', '');
            
            if ~strcmpi(read, '')
                % Remove trailing separator and split the string
                read = strtrim(read);
                adcDataStrings = strsplit(read, separator);
                % Convert strings to double and store in array
                adcData = str2double(adcDataStrings);
                
                % Initialize arrays to store odd and even values
                voltageData = [];
                currentData = [];
                % Iterate through the adcData array
                for i = 1:numel(adcData)
                    if mod(i, 2) == 1
                        % If index is odd, append the value tovoltageData
                        voltageData(end+1) = adcData(i);
                    else
                        % If index is even, append the value tocurrentData
                        currentData(end+1) = adcData(i);
                    end
                end
                
                obj.measuredData;
                isDataReady = true;
                
                % Check if the filename is provided
                if nargin > 1 && ~isempty(filename)
                    % Store the data to a .mat file if filename is provided
                    save(filename, 'adcData');
                    disp(['Data received and stored in ' filename]);
                else
                    disp('No filename provided, data not saved.');
                end
            end
        end
        
        % ***************** Calculation methods ********************** %
        % Function to calculate various parameters from voltage and current data
        function measuredData = calculate(obj, voltageData, currentData)
            % Store buffers
            % If parameter empty use already stored in obj
            if isempty(voltageData)
                voltageData = obj.measuredData.voltageData;
            else
                obj.measuredData.voltageData = voltageData;
            end
            if isempty(currentData)
                currentData = obj.measuredData.currentData;
            else
                obj.measuredData.currentData = currentData;
            end

            % Calculate max values
            obj.measuredData.maxVoltage = max(voltageData);
            obj.measuredData.maxCurrent = max(currentData);
            
            % Calculate RMS values
            obj.measuredData.rmsVoltage = sqrt(mean(voltageData.^2));
            obj.measuredData.rmsCurrent = sqrt(mean(currentData.^2));
            
            % Find zero crossings
            obj.measuredData.zeroCrossVoltage = obj.zeroCross(voltageData);
            obj.measuredData.zeroCrossCurrent = obj.zeroCross(currentData);

            % Calculate frequency of voltage and current
            obj.measuredData.frequencyVoltage = obj.calculateFrequency(obj.measuredData.zeroCrossVoltage, obj.samplingFrequency);
            obj.measuredData.frequencyCurrent = obj.calculateFrequency(obj.measuredData.zeroCrossCurrent, obj.samplingFrequency);
            
            % Calculate phase shift between voltage and current
            obj.measuredData.phaseShift = obj.calculatePhaseShift(obj.measuredData.zeroCrossVoltage, obj.measuredData.zeroCrossCurrent, obj.samplingFrequency);
            
            % Convert phase shift to radians
            phaseShiftRadians = deg2rad(obj.measuredData.phaseShift);
            % Calculate impedance with phase shift
            obj.measuredData.impedance = (obj.measuredData.maxVoltage / obj.measuredData.maxCurrent) * exp(1i * phaseShiftRadians);

            measuredData = obj.measuredData;
        end
    end
    
    methods (Static)
        % Function to calculate frequency from signal data
        function frequency = calculateFrequency(zeroCrossIndices, samplingFrequency)
            % Calculate frequency
            frequency = 0;
            if length(zeroCrossIndices) > 1
                period = mean(abs(diff(zeroCrossIndices))) / samplingFrequency;
                frequency = 1 / period;
            end
        end

        % Function to calculate phase shift between two signals
        function phaseShift = calculatePhaseShift(zeroCrossIndicesSignal1, zeroCrossIndicesSignal2, samplingFrequency)
            % Trim the longer zero crossings array
            minLength = min(length(zeroCrossIndicesSignal1), length(zeroCrossIndicesSignal2));
            zeroCrossIndicesSignal1 = zeroCrossIndicesSignal1(1:minLength);
            zeroCrossIndicesSignal2 = zeroCrossIndicesSignal2(1:minLength);
            
            % Calculate number of samples between corresponding zero crossings
            nSamplesDifferences = zeroCrossIndicesSignal2 - zeroCrossIndicesSignal1;

            % Combine zero crossings indices into one array
            zeroCrossIndices = sort([zeroCrossIndicesSignal1, zeroCrossIndicesSignal2]);
            % Calculate number of samles per signal period
            nSamplesPerPeriod = mean(abs(diff(zeroCrossIndices)));
            
            % phase error compensation
            phaseCompensation = 0;
            % Calculate phase shift in degrees
            phaseShift = 180 * (nSamplesDifferences / nSamplesPerPeriod) - phaseCompensation;
            % Shift the phase if it exceeds limits (-180° - 180°)
            % also addressing of zero crossing index have no neer counter
            % index in reach of -180° to 180°. Checking the next one out of
            % reach and shifting the measured phase by 360°
            phaseShift = mod(phaseShift + 180, 360) - 180;
            
            phaseShift = mean(phaseShift);
        end

        % Function to find zero crossings indices of a signal
        function indices = zeroCross(signal)
            % Get the length of the signal
            N = size(signal, 2);
            % Create a time vector
            t = 1:N;
            
            % Find the points where the signal is above zero
            fAbove = signal .* (signal >= 0);
            % Find the indices where the signal changes from below zero to above zero
            fCrossRaw = find(diff(fAbove > 0));
            
            % Initialize an empty array to store the indices of zero crossings
            indices = [];
            
            % Loop through the found crossing points
            for k1 = 1:size(fCrossRaw, 2) - 1
                % Fit a line near the zero-crossing points
                b = [[1; 1] [t(fCrossRaw(k1)); t(fCrossRaw(k1) + 1)]] \ [signal(fCrossRaw(k1)); signal(fCrossRaw(k1) + 1)];
                % Calculate the zero-crossing point
                zeroCrossing = -b(1) / b(2);
                % Check if the signal is going upwards (from negative to positive)
                if b(1) < 0
                    % Append the zero-crossing index to the array
                    indices(end + 1) = zeroCrossing;
                end
            end
        end  
    end
end