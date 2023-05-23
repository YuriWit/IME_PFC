clear; close all; clc;
addpath("Classes\")
rng(2023);

% Global Params 
% gp for globalParams
gp.c = 3e8; % speed of light (m/s)
globalParams = gp;

% Simple Radar 
% rp for radarParams
rp.Position = [0;0;0]; % position vector (m)
rp.fc = 10e9; % sweep central frequency (Hz)
rp.B = 300e6; % sweep bandwidth (Hz)
rp.fs = 15e9; % sample rate (Hz) 
rp.T = 3e-7; % sweep time (s)
rp.prf = 3e3; % pulse repetition frequency (Hz)
rp.nPulses = 1; % number of pulses
radarParams = rp;

% Point Target 
% tp for targetParams
tp.Position = [-20;0;0]; % position vector (m)
tp.Velocity = [12;0;0]; % velocity vector (m/s)
tp.meanRCS = 10; % mean radar cross section (m^2)
tp.bRCS = 10;
tp.bladeRadarCrossSection = .1;
tp.numberOfBlades = 4;
tp.bladeLength = 6.5;
tp.bladeAngularVelocity = [0;0;240*2*pi/60];
targetParams = tp;

% Enviorment(FreeSpace)
%add class for env

%% Initiate Objects
radar = Radar(params);
target = RadarTarget(params);
enviorment = phased.FreeSpace(...
    'PropagationSpeed',params.c,...
    'OperatingFrequency',params.sweepCentralFrequency,...
    'TwoWayPropagation',true,...
    'SampleRate',params.sampleRate);

%% Transmit
NSampPerPulse = round(params.sampleRate/params.pulseRepetitionFrequency);
Niter = 16;
y = complex(zeros(NSampPerPulse,Niter));
dt = 1/params.pulseRepetitionFrequency;
for i=1:Niter
    [scattersPosition,scattersVelocity] = target.targetMotion(dt);
    [~,scattersAngle] = rangeangle(scattersPosition, radar.Position);
    
    transmittedSignal = radar.getTransmittedSignal(scattersAngle);
    %% Free Space Propagation
    propagatedSignal = enviorment(...
        transmittedSignal,...
        radar.Position,...
        scattersPosition,...
        radar.Velocity,...
        scattersVelocity);
    
    %% Target Reflection
    %test
    reflectedSignal = target.getReflectedSignal(propagatedSignal);
    
    %% Reception
    receivedSignal = radar.receiveReflectedSignal(...
        reflectedSignal,...
        scattersAngle);
    
    %% Matched Filter
    integratedSignal = sum(reflectedSignal, 2);
    y(:,i) = integratedSignal;

end

rangeDopplerResponse = phased.RangeDopplerResponse(...
    'PropagationSpeed',params.c,...
    'SampleRate',params.sampleRate,...
    'SweepSlope',params.sweepBandwidth/params.sweepTime,...
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',128,...
    'DopplerOutput','Speed',...
    'OperatingFrequency',params.sweepCentralFrequency,...
    'DechirpInput',false,...
    'PRF',params.pulseRepetitionFrequency);
matchedFilderCoefficients = getMatchedFilter(radar.WaveForm);

%% Plots
figure
spectrogram(...
    transmittedSignal,...
    hamming(32),...
    30,...
    [],...
    params.sampleRate,...
    'centered',...
    'yaxis');
xlim([0 30])
%ylim([0 2])

figure
plotResponse(...
    rangeDopplerResponse,...
    y(:,1:Niter),...
    matchedFilderCoefficients);
ylim([0 100])











