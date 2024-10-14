function plotCharacteristics(frequencyCharacteristic)
    % Initialize arrays to store data
    frequencyVoltage = [];
    maxVoltage = [];
    phaseShift = [];
    rmsVoltage = [];

    if isfield(frequencyCharacteristic, 'frequencyCharacteristic')
        frequencyCharacteristic = frequencyCharacteristic.frequencyCharacteristic;
    end
    frequencyCharacteristic = frequencyCharacteristic(1:end-2);
    % Iterate through each element in frequencyCharacteristic
    for i = 1:numel(frequencyCharacteristic)
        % Append or truncate data to ensure consistent lengths
        measuredData = frequencyCharacteristic(i);
        frequencyVoltage = [frequencyVoltage; measuredData.frequencyVoltage];
        maxVoltage = [maxVoltage; measuredData.maxVoltage];
        rmsVoltage = [rmsVoltage; measuredData.rmsVoltage];
        phaseShift = [phaseShift; measuredData.phaseShift];
    end

    % Plot the frequency characteristic
    figure(2);
    clf;
    yyaxis left;
    loglog(frequencyVoltage, rmsVoltage, '-o');
    ylabel('Voltage [V]');
    hold on;
    
    yyaxis right;
    plot(frequencyVoltage, phaseShift, '-o');
    ylabel('Phase Shift [°]');
    
    xlabel('Frequency [Hz]');
    title('Frequency Characteristic');
    grid on;
        
    % Set x-axis limits from DC to 30 MHz
    xlim([1, 30e6]);
end