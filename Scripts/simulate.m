%% Run Simulation
tStart = tic;

receivedSignal = zeros(length(radar.Waveform()),numPulses);
transmittedSignal = zeros(length(radar.Waveform()),numPulses);
dt = 1/prf;
for i=1:numPulses
    % update motion
    target.update(dt)
    pointTargets = target.getPointTargets();

    for j=1:length(pointTargets)
        % get specific point target
        pTarget = pointTargets(j);

        % get range and angle
        [targetRange,targetAngle] = rangeangle(pTarget.Position,radar.Position);
    
        % signal transmission
        transmittedSignal(:,i) = radar.getTransmittedSignal(targetAngle);
    
        % signal propagation
        propagatedSignal = enviroment(...
            transmittedSignal(:,i),...
            radar.Position,...
            pTarget.Position,...
            radar.Velocity,...
            pTarget.Velocity);
        
        % signal reflection
        reflectedSignal = pTarget.getReflectedSignal(propagatedSignal);
        
        % signal reception
        receivedSignal(:,i) = receivedSignal(:,i) + ...
            radar.receiveReflectedSignal(...
                reflectedSignal,...
                targetAngle);
    end
end

tEnd = toc(tStart)