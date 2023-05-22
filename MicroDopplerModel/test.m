clear; close all; clc;
addpath("Classes\")
rng(2023);

%% Params
params.c = 3e8;

% Radar
params.radarPosition = [0;0;0];
params.radarVelocity = [0;0;0];

% Waveform
params.sweepCentralFrequency = 1e9;
params.sweepBandwidth = 300e6; % Sweep bandwidth in Hz
params.sampleRate = 6e9; % Sample rate in Hz
params.sweepTime = 1e-6; % Sweep time in seconds
params.pulseRepetitionFrequency = 1e4; %kHz
params.numberOfPulses = 1;

% Target (Helicopter)
params.targetPosition = [100;0;0];
params.targetVelocity = [50;0;0];
params.meanRCS = 10;
params.bodyRadarCrossSection = 10;
params.bladeRadarCrossSection = .1;
params.numberOfBlades = 4;
params.bladeLength = 6.5;
params.bladeAngularVelocity = [0;0;240*2*pi/60];

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
NSampPerPulse = round(100*params.sampleRate*params.sweepTime);
Niter = 32;
y = complex(zeros(NSampPerPulse,Niter));
dt = 1/params.pulseRepetitionFrequency;
for i=1:Niter
    [scattersPosition,scattersVelocity] = target.targetMotion(dt);
    [~,scattersAngle] = rangeangle(scattersPosition, radar.Position);
    targetDistance = sqrt(sum((scattersPosition - radar.Position).^2));
    
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
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',128,...
    'DopplerOutput','Speed',...
    'OperatingFrequency',params.sweepCentralFrequency);
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
    xlim([0 1])
    ylim([-2 3])

figure
plotResponse(...
    rangeDopplerResponse,...
    y(:,1:Niter),...
    matchedFilderCoefficients);
ylim([0 100])











