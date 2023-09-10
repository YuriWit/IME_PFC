tStart = tic;
%% Run Simulation

numPulses = 2048;
receivedSignal = zeros(length(radar.Waveform()),numPulses);
transmittedSignal = zeros(length(radar.Waveform()),numPulses);
dt = 1/rp.prf;
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
%% Process and Plot

filter = getMatchedFilter(radar.Waveform);
mf = phased.MatchedFilter('Coefficients', filter);

tymf = mf(transmittedSignal);
iymf = sum(abs(tymf'))';
[~,tmaxi] = max(iymf);

% time doppler map
figure;
ymf = mf(receivedSignal);
[~,ridx] = max(sum(abs(ymf),2));
[p,f,t] = pspectrum(ymf(ridx,:),rp. ...
    prf,'spectrogram');
imagesc( t/1e-3, dop2speed(f,c/fc)/2, pow2db(p), [-30 -28]);
colorbar
ylim([-100 100])
xlim([50 100])
xlabel('Tempo [ms]');
ylabel('Velocidade [m/s]');
title('Mapa Tempo Doppler');

% Range Doppler Response
figure;
rangeDopplerResponse = phased.RangeDopplerResponse(...
    'PropagationSpeed', rp.c,...
    'SampleRate',rp.fs,...
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',1024,...
    'DopplerOutput','Speed',...
    'OperatingFrequency',rp.fc);
filter = getMatchedFilter(radar.Waveform);
plotResponse(...
    rangeDopplerResponse,...
    receivedSignal(:,1:numPulses),...
    filter);
ylim([0 500])
xlim([-100 100])

%Doppler response
figure;
ymf = mf(receivedSignal);
[~,indMax] = max(abs(ymf(:,1)));
N = length(ymf(indMax,:));
fshift = (-N/2:N/2-1)*(rp.prf/N);
Y = fftshift(fft(ymf(indMax,:)));
Y = abs(Y).^2 / N;
speed = dop2speed(fshift, c/fc)/2;
plot(speed,Y);
xlabel('Frequency [Hz]');
ylabel('Power[dB]');
title('Doppler response');
xlim([-75 75])