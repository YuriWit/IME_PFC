%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test Case 4
% Two Blade Helicopter Target with center stationary
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Start
clear; close all; clc;
addpath("Classes\")
addpath("Functions\")
rng(2023);

%% Parameters

% Global Params 
c = physconst('LightSpeed'); % speed of light (m/s)
fc = 3e9; %    central frequency (Hz)
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

% Two Blade Helicopter Target
% tp for targetParams
tp.c = c;
tp.fc = fc;
tp.meanBodyRCS = 10; % mean radar cross section (m^2)
tp.meanBladeRCS = 1; % mean radar cross section (m^2)
tp.radiusVector = [1;0;0]; % radius vector (m)
tp.angularVelocityVector = [0;0;955]*2*pi/60; % angular velocity vector (rpm)

tp.position = [-100;0;0]; % position vector (m)
tp.velocity = [0;0;0]; % velocity vector (m/s)

%% Initiate Objects
radar = SimpleRadar(rp);
target = HelicopterTarget(tp);
enviroment = phased.FreeSpace(...
    'PropagationSpeed',c,...
    'OperatingFrequency',fc,...
    'TwoWayPropagation',true,...
    'SampleRate',fs);

%% Transmit
numPulses = 1e4;
receivedSignal = zeros(length(radar.Waveform()),numPulses);
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
        transmittedSignal = radar.getTransmittedSignal(targetAngle);
    
        % signal propagation
        propagatedSignal = enviroment(...
            transmittedSignal,...
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

%% Plots
filter = getMatchedFilter(radar.Waveform);
mf = phased.MatchedFilter('Coefficients', filter);
ymf = mf(receivedSignal);

% doppler response
figure;
t = (1:1:length(ymf(:,1)))*fs;
plot(t, log10(abs(ymf(:,1)).^2));
xlabel('Frequency [Hz]');
ylabel('h(f) in dB');
title('Doppler response');

% slow time response
figure;
t = (1:1:length(ymf(1,:)))*fs;
plot(t, 10*log10(abs(fft(ymf')).^2));
xlabel('Time [ms]');
ylabel('h(t)');
title('Slow time response');

% spectrum
figure;
xlabel('Frequency [MHz]');
ylabel('Power[dBFS]');
title('Spectrum');

% stft
figure;
xlabel('Time [ms]');
ylabel('Doppler velocity [m/s]');
title('Doppler response');




% range doppler response
figure;
rangeDopplerResponse = phased.RangeDopplerResponse(...
    'PropagationSpeed', rp.c,...
    'SampleRate',rp.fs,...
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',128,...
    'DopplerOutput','Speed',...
    'OperatingFrequency',rp.fc);
plotResponse(...
    rangeDopplerResponse,...
    receivedSignal(:,1:numPulses),...
    filter);
ylim([0 1000])
xlim([-100 100])

% spectrograma
figure;
filter = getMatchedFilter(radar.Waveform);
mf  = phased.MatchedFilter('Coefficients',filter);
ymf = mf(receivedSignal');
[~,ridx] = max(sum(abs(ymf),2)); 
pspectrum(ymf(ridx,:),rp.prf,'spectrogram')


% % plot radar signal
% signal = real(radar.Waveform());
% nSamples = rp.fs*rp.T;
% figure
% plot(signal(1:nSamples))
% 
% signal = radar.getTransmittedSignal(0);
% figure
% spectrogram(signal(1:nSamples),hamming(64),60,[],rp.fc,'yaxis');

