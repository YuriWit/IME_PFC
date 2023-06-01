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
p.radarVelocity = [0;0;0]; % velocity vector (m/s)
p.fs = 20e6; % sample rate (Hz) 
p.fc = 3e9; %    central frequency (Hz)
p.B = 5e6; % sweep bandwidth (Hz)
p.T = 10e-6; % sweep time (s)
p.prf = 1e4; % pulse repetition frequency (Hz)
p.nPulses = 1; % number of pulses

% Point Target 
% tp for targetParams
p.targetPosition = [-500;0;0]; % position vector (m)
p.targetVelocity = [50;0;0]; % velocity vector (m/s)
p.meanRCS = 1; % mean radar cross section (m^2)
p.p1.Position = [100;0;0];
p.p1.Velocity = [50;0;0];
p.p2.Position = [-100;0;0];
p.p2.Velocity = [-25;0;0];

params = p;

%% Initiate Objects
radar = SimpleRadar(params);
target = SimpleTarget(params);
enviroment = phased.FreeSpace(...
    'PropagationSpeed',p.c,...
    'OperatingFrequency',p.fc,...
    'TwoWayPropagation',true,...
    'SampleRate',p.fs);

%% Test Radar
if false
    signal = real(radar.Waveform());
    nSamples = p.fs*p.T;
    figure
    plot(signal(1:nSamples))

    signal = radar.getTransmittedSignal(0);
    figure
    spectrogram(signal(1:nSamples),hamming(64),60,[],p.fc,'yaxis');
end
%% Transmit
numPulses = 64;
receivedSignal = zeros(length(radar.Waveform()),numPulses);
dt = 1/p.prf;
for i=1:numPulses
    % update bodies motion
    target.update(dt)
    
    pointTargets = target.getPointTargets();
    for j=1:length(pointTargets)
        pTarget = pointTargets(j);

        % get global values
        globalTargetPosition = target.Position + pTarget.Position;
        globalTargetVelocity = target.Velocity + pTarget.Velocity;
        gtPosition = globalTargetPosition;
        gtVelocity = globalTargetVelocity;

        % get revelant values
        [targetRange,targetAngle] = rangeangle(gtPosition,radar.Position);
    
        % signal transmission
        transmittedSignal = radar.getTransmittedSignal(targetAngle);
    
        % signal propagation
        propagatedSignal = enviroment(...
            transmittedSignal,...
            radar.Position,...
            gtPosition,...
            radar.Velocity,...
            gtVelocity);
        
        % signal reflection
        reflectedSignal = pTarget.getReflectedSignal(propagatedSignal);
        
        % signal reception
        receivedSignal(:,i) = receivedSignal(:,i) + ...
            radar.receiveReflectedSignal(...
                reflectedSignal,...
                targetAngle);
    end
end
rangeDopplerResponse = phased.RangeDopplerResponse(...
    'PropagationSpeed', p.c,...
    'SampleRate',p.fs,...
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',128,...
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
xlim([-100 100])











