%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test Case 2
% Spinning Point Target with center stationary
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Start
clear; close all; clc;
addpath("Classes\")
addpath("Functions\")
rng(2023);

%% Parameters

% Global Params 
c = physconst('LightSpeed'); % speed of light (m/s)
fc = 3.7e9; % central frequency (Hz)
fs = 20e6; % sample rate (Hz) 

% Simple Radar 
% rp for radarParams
rp.c = c;
rp.fc = fc;
rp.fs = fs;
rp.B = 5e6; % sweep bandwidth (Hz)
rp.T = 10e-6; % sweep time (s)
rp.prf = 2e4; % pulse repetition frequency (Hz)
rp.nPulses = 1; % number of pulses
rp.position = [0;0;0]; % position vector (m)
rp.velocity = [0;0;0]; % velocity vector (m/s)

% Spinning Point Target
% tp for targetParams
tp.c = c;
tp.fc = fc;
tp.meanRCS = 1; % mean radar cross section (m^2)
tp.radiusVector = [0;.331/2;0]; % radius vector (m)
tp.angularVelocityVector = [0;0;3000]*2*pi/60; % angular velocity vector (rpm)

tp.position = [-250;0;0]; % position vector (m)
tp.velocity = [0;0;0]; % velocity vector (m/s)

%% Initiate Objects
radar = SimpleRadar(rp);
target = SpinningPointTarget(tp);
enviroment = phased.FreeSpace(...
    'PropagationSpeed',c,...
    'OperatingFrequency',fc,...
    'TwoWayPropagation',true,...
    'SampleRate',fs);

%% Transmit
numPulses = 2048;
receivedSignal = zeros(length(radar.Waveform()),numPulses);
transmittedSignal = zeros(length(radar.Waveform()),numPulses);
dt = 1/rp.prf;
for i=1:numPulses
    % update bodies motion
    target.update(dt)
    
    pointTargets = target.getPointTargets();
    for j=1:length(pointTargets)
        pTarget = pointTargets(j);

        % get revelant values
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

%% prossessing
filter = getMatchedFilter(radar.Waveform);
mf = phased.MatchedFilter('Coefficients', filter);

tymf = mf(transmittedSignal);
iymf = sum(abs(tymf'))';
[~,tmaxi] = max(iymf);

% time doppler map
figure;
ymf = mf(receivedSignal);
[~,ridx] = max(sum(abs(ymf),2));
[p,f,t] = pspectrum(ymf(ridx,:),rp.prf,'spectrogram');
imagesc( t/1e-3, dop2speed(f,c/fc)/2, pow2db(p));
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
signal = ymf(indMax,:);
N = length(signal);
fshift = (-N/2:N/2-1)*(rp.prf/N);
Y = fftshift(fft(signal));
Y = abs(Y).^2 / N;
speed = dop2speed(fshift, c/fc)/2;
plot(speed,Y);
xlabel('Speed [m/s]');
ylabel('Power');
title('Doppler response');
xlim([-75 75])
