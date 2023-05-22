clear; close all; clc;
addpath("Classes\")

%% Params
params.c = 3e8;

% Radar
params.radarPosition = [0;0;0];
params.radarVelocity = [0;0;0];

% Waveform
params.sweepCentralFrequency = 1e9;
params.sweepBandwidth = 300e6; % Sweep bandwidth in Hz
params.sampleRate = 6e9; % Sample rate in Hz
params.sweepTime = 1e-4; % Sweep time in seconds
params.pulseRepetitionFrequency = 1e3; %kHz
params.numberOfPulses = 32;

% Target (Helicopter)
params.targetPosition = [100;100;0];
params.targetVelocity = [120;0;0];
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
figure
plot(radar.WaveForm)
return
target = RadarTarget(params);
enviorment = phased.FreeSpace(...
    'PropagationSpeed',params.c,...
    'OperatingFrequency',params.sweepCentralFrequency,...
    'TwoWayPropagation',true,...
    'SampleRate',params.sampleRate);

%% Transmit
NSampPerPulse = round(...
    params.sampleRate/params.pulseRepetitionFrequency);
%Niter = 10e3;
Niter = 32;
y = complex(zeros(NSampPerPulse*params.numberOfPulses,Niter));
rng(2023);
for i=1:Niter
    dt = 1/params.pulseRepetitionFrequency;
    [scattersPosition,scattersVelocity] = target.targetMotion(dt);
    [~,scattersAngle] = rangeangle(target.Position, radar.Position);
    targetDistance = sqrt(sum((target.Position - radar.Position).^2));
    
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
    xlim([2 8])
    ylim([-5 5])

figure
plotResponse(...
    rangeDopplerResponse,...
    y,...
    matchedFilderCoefficients);
ylim([0 100])











