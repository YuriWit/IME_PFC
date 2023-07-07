clear; close all; clc;
addpath("Classes\")
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
rp.prf = 1e4; % pulse repetition frequency (Hz)
rp.nPulses = 1; % number of pulses
rp.position = [0;0;0]; % position vector (m)
rp.velocity = [0;0;0]; % velocity vector (m/s)

% Helicopter Target 
% tp for targetParams
tp.c = c;
tp.fc = fc;
tp.meanBodyRCS = 1; % mean body cross section (m^2)
tp.meanBladeRCS = .1; % mean blase cross section (m^2)
tp.radiusVector = [0;1;0]; % radius vector (m^3)
tp.angularVelocityVector = [0;0;400] *2*pi/60; % angular velocity vector (rad/s)
tp.position = [-500;-1;0]; % position vector (m)
tp.velocity = [0;0;0]; % velocity vector (m/s)

%% Initiate Objects
radar = SimpleRadar(rp);
target = HelicopterTarget(tp);
enviroment = phased.FreeSpace(...
    'PropagationSpeed',c,...
    'OperatingFrequency',fc,...
    'TwoWayPropagation',true,...
    'SampleRate',fs);

%% Test Radar
if false
    signal = real(radar.Waveform());
    nSamples = rp.fs*rp.T;
    figure
    plot(signal(1:nSamples))

    signal = radar.getTransmittedSignal(0);
    figure
    spectrogram(signal(1:nSamples),hamming(64),60,[],rp.fc,'yaxis');
end
%% Transmit
numPulses = 64;
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
rangeDopplerResponse = phased.RangeDopplerResponse(...
    'PropagationSpeed', rp.c,...
    'SampleRate',rp.fs,...
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',128,...
    'DopplerOutput','Speed',...
    'OperatingFrequency',rp.fc);

filter = getMatchedFilter(radar.Waveform);

%% Plots


figure
plotResponse(...
    rangeDopplerResponse,...
    receivedSignal(:,1:numPulses),...
    filter);
ylim([0 1000])
xlim([-100 100])











