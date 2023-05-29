clear; close all; clc;
addpath("Classes\")
rng(2023);

%% Parameters

% Global Params 
% gp for globalParams
p.c = physconst('LightSpeed'); % speed of light (m/s)

% Simple Radar 
% rp for radarParams
p.radarPosition = [0;0;0]; % position vector (m)
p.fs = 20e6; % sample rate (Hz) 
p.fc = 3e9; %    central frequency (Hz)
p.B = 5e6; % sweep bandwidth (Hz)
p.T = 10e-6; % sweep time (s)
p.prf = 1e3; % pulse repetition frequency (Hz)
p.nPulses = 1; % number of pulses

% Point Target 
% tp for targetParams
p.targetPosition = [500;0;0]; % position vector (m)
p.targetVelocity = [200;0;0]; % velocity vector (m/s)
p.meanRCS = 10; % mean radar cross section (m^2)

params = p;

%% Initiate Objects
radar = SimpleRadar(params);
target = PointTarget(params);
enviroment = phased.FreeSpace(...
    'PropagationSpeed',p.c,...
    'OperatingFrequency',p.fc,...
    'TwoWayPropagation',true,...
    'SampleRate',p.fs);

%% Test Radar
if true
    signal = real(radar.Waveform());
    nSamples = p.fs*p.T;
    figure
    plot(signal(1:nSamples))

    signal = radar.getTransmittedSignal(0);
    figure
    spectrogram(signal(1:nSamples),hamming(64),60,[],p.fc,'yaxis');
end
%% Transmit
numPulses = 16;
receivedSignal = zeros(length(radar.Waveform()),numPulses);
dt = 1/p.prf;
for i=1:numPulses
    % update bodies motion
    target.update(dt)
    
    % get revelant values
    [targetRange,targetAngle] = rangeangle(target.Position,radar.Position);

    % signal transmission
    transmittedSignal = radar.getTransmittedSignal(targetAngle);

    % signal propagation
    propagatedSignal = enviroment(...
        transmittedSignal,...
        radar.Position,...
        target.Position,...
        radar.Velocity,...
        target.Velocity);
    
    % signal reflection
    reflectedSignal = target.getReflectedSignal(propagatedSignal);
    
    % signal reception
    receivedSignal(:,1) = radar.receiveReflectedSignal(...
        reflectedSignal,...
        targetAngle);
end
rangeDopplerResponse = phased.RangeDopplerResponse(...
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',128,...
    'SampleRate',p.fs,...
    'DopplerOutput','Speed',...
    'OperatingFrequency',p.fc);

filter = getMatchedFilter(radar.Waveform);

%% Plots


figure
plotResponse(...
    rangeDopplerResponse,...
    receivedSignal(:,1:numPulses),...
    filter);
ylim([0 1000])
vlim([-300 300])











