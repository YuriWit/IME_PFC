%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test Case 3
% Two Blade Rotor Target with center stationary
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

% Two Blade Rotor Target
% tp for targetParams
tp.c = c;
tp.fc = fc;
tp.meanBladeRCS = 1; % mean radar cross section (m^2)
tp.radiusVector = [1;0;0]*0.0331/2; % radius vector (m)
tp.angularVelocityVector = [0;0;3000]*2*pi/60; % angular velocity vector (rpm)

tp.position = [-100;0;0]; % position vector (m)
tp.velocity = [0;0;0]; % velocity vector (m/s)

%% Initiate Objects
radar = SimpleRadar(rp);
target = TwoBladeRotorTarget(tp);
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

% fast time response
figure(1);
t = (1:1:length(ymf(:,1)))/fs;
plot(t, 10*log10(abs(ymf(:,1)).^2));
xlabel('Time [s]');
ylabel('CCF in dB');
title('Fast time response');

% slow time response
figure(2);
FFTymf = fft(ymf');%FFT of each column
maxFFTymf = max(abs(FFTymf));
plot(t, 10*log10(abs(maxFFTymf).^2));
xlabel('Time [ms]');
ylabel('h(t)');
title('Slow time response');

% spectrum
figure(3);
[~,indMax] = max(abs(ymf(:,1)));
plot(linspace(-fs/2,fs/2,rp.T*fs)/1e6,10*log10(abs(fftshift(fft(receivedSignal(indMax-rp.T*fs+1:indMax,1))))))
xlabel('Frequency [MHz]');
ylabel('Power[dBFS]');
title('Spectrum');

%Doppler response
figure(4);
plot(10*log10(abs(fftshift(fft(receivedSignal(1:indMax,1))))))
xlabel('Frequency [Hz]');
ylabel('Power[dB]');
title('Doppler response');

% range doppler response
figure(5);
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

