clear; close all; clc;
addpath("Classes\")
rng(2023);

%% Parameters

% Global Params 
% gp for globalParams
p.c = 3e8; % speed of light (m/s)

% Simple Radar 
% rp for radarParams
p.radarPosition = [0;0;0]; % position vector (m)
p.fc = 10e9; % sweep central frequency (Hz)
p.B = 300e6; % sweep bandwidth (Hz)
p.fs = 15e9; % sample rate (Hz) 
p.T = 3e-7; % sweep time (s)
p.prf = 3e3; % pulse repetition frequency (Hz)
p.nPulses = 1; % number of pulses

% Point Target 
% tp for targetParams
p.targetPosition = [500;0;0]; % position vector (m)
p.targetVelocity = [20;0;0]; % velocity vector (m/s)
p.meanRCS = 1; % mean radar cross section (m^2)

params = p;

%% Initiate Objects
radar = SimpleRadar(params);
target = PointTarget(params);
enviorment = phased.FreeSpace(...
    'PropagationSpeed',p.c,...
    'OperatingFrequency',p.fc,...
    'TwoWayPropagation',true,...
    'SampleRate',p.fs);

%% Transmit
numPulses = 16;
receivedSignal = zeros(length(radar.Waveform()),numPulses);
dt = 1/p.prf;
for i=1:numPulses
    % update bodies motion
    %radar.update(dt)
    %target.update(dt)
    
    % get revelant values
    [targetRange,targetAngle] = rangeangle(target.Position,radar.Position);

    % signal transmission
    transmittedSignal = radar.getTransmittedSignal(targetAngle);

    % signal propagation
    propagatedSignal = enviorment(...
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
    'OperatingFrequency',p.fc,...
    'PRFSource','Property',...
    'PRF',p.prf);

filter = getMatchedFilter(radar.Waveform);

%% Plots
figure
spectrogram(...
    transmittedSignal,...
    hamming(32),...
    30,...
    [],...
    p.fs,...
    'centered',...
    'yaxis');
%xlim([0 30])
%ylim([0 2])

figure
plotResponse(...
    rangeDopplerResponse,...
    receivedSignal(:,1:numPulses),...
    filter);
%ylim([0 100])











