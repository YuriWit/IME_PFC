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
rp.prf = 4e4; % pulse repetition frequency (Hz)
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

tp.position = [100;0;0]; % position vector (m)
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
numPulses = 4096;
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


% time doppler
figure;
ymf = mf(receivedSignal);
[~,ridx] = max(sum(abs(ymf),2));
pspectrum(ymf(ridx,:),rp.prf,'spectrogram')
return

% base sample delay
tymf = mf(transmittedSignal);
iymf = sum(abs(tymf'))';
[~,tmaxi] = max(iymf);

% integrated signal to mesure distance
rymf = mf(receivedSignal);
iymf = sum(abs(rymf'))';
[~,rmaxi] = max(iymf);
distance = (tmaxi - rmaxi) / fs * c / 2;
figure;
t = (1:1:length(iymf)-tmaxi) / fs * c / 2 / 1e3;
plot(t, iymf(tmaxi+1:end));
xlim([0 5]);
xlabel("distance [km]")
ylabel("magnitude")

% range doppler to mesure speed
max_speed = dop2speed(rp.prf/2,c/fc)/2;
speed_res = 2*max_speed/numPulses;

slowTimeFFT = abs(fftshift(fft(receivedSignal(rmaxi,:))));
[~,stfftmaxi] = max(slowTimeFFT);
figure;
n = length(slowTimeFFT);
t = linspace(-max_speed,max_speed,n);
plot(t, slowTimeFFT);


figure;
fftsize = 2048;
rangeDoppler = zeros(fftsize,numPulses-fftsize);
pulse_samples = floor(rp.T*fs);
for i=1:numPulses-fftsize
    rangeDoppler(:,i) = abs(fftshift(fft(receivedSignal(rmaxi-pulse_samples,i:i+fftsize-1)'))).^2;
end
x = linspace(0,(numPulses-fftsize)/rp.prf, numPulses-fftsize)/1e-3;
y = linspace(-max_speed, max_speed, fftsize);
max_val = max(max(rangeDoppler));
min_val = min(min(rangeDoppler));

image(x,y,1e13*rangeDoppler);
xlim([0 120]);
ylim([-50 50]);
colorbar
beep