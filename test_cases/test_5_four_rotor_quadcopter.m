%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test Case 5
% Four Rotor Quadcopter Target with center stationary
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Start
clear; close all; clc;
addpath("Classes\")
addpath("Functions\")
rng(2023);

%% Parameters

% Global Params 
c = physconst('LightSpeed'); % speed of light (m/s)
fc = 3.7e9; %    central frequency (Hz)
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

% Four Rotor Quadcopter Target
% tp for targetParams
tp.c = c;
tp.fc = fc;
tp.meanBodyRCS = 1; % mean radar cross section (m^2)
tp.meanBladeRCS = 1; % mean radar cross section (m^2)\

tp.rotor1RelativePosition = .15*[1;1;0]; % relative position velocity vector (m/s)
tp.rotor2RelativePosition = .15*[-1;1;0]; % relative position velocity vector (m/s)
tp.rotor3RelativePosition = .15*[-1;-1;0]; % relative position velocity vector (m/s)
tp.rotor4RelativePosition = .15*[1;-1;0]; % relative position velocity vector (m/s)

r = .331/2;
a1 = 30;
a2 = 60;
a3 = 90;
a4 = 120;
tp.rotor1RadiusVector = r*[cos(a1*pi/180);sin(a1*pi/180);0]; % radius vector (m^3)
tp.rotor2RadiusVector = r*[cos(a2*pi/180);sin(a2*pi/180);0]; % radius vector (m^3)
tp.rotor3RadiusVector = r*[cos(a3*pi/180);sin(a3*pi/180);0]; % radius vector (m^3)
tp.rotor4RadiusVector = r*[cos(a4*pi/180);sin(a4*pi/180);0]; % radius vector (m^3)

tp.rotor1AngularVelocityVector = [0;0;3000] *2*pi/60; % angular velocity vector (rad/s)
tp.rotor2AngularVelocityVector = [0;0;1e-9] *2*pi/60; % angular velocity vector (rad/s)
tp.rotor3AngularVelocityVector = [0;0;1e-9] *2*pi/60; % angular velocity vector (rad/s)
tp.rotor4AngularVelocityVector = [0;0;1e-9] *2*pi/60; % angular velocity vector (rad/s)

tp.position = [-250;0;0]; % position vector (m)
tp.velocity = [0;0;0]; % velocity vector (m/s)

%% Initiate Objects
radar = SimpleRadar(rp);
target = QuadcopterTarget(tp);
enviroment = phased.FreeSpace(...
    'PropagationSpeed',c,...
    'OperatingFrequency',fc,...
    'TwoWayPropagation',true,...
    'SampleRate',fs);

%% Transmit
numPulses = 2048;
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

tymf = mf(transmittedSignal);
iymf = sum(abs(tymf'))';
[~,tmaxi] = max(iymf);

% time doppler map
figure;
ymf = mf(receivedSignal);
[~,ridx] = max(sum(abs(ymf),2));
[p,f,t] = pspectrum(ymf(ridx,:),rp. ...
    prf,'spectrogram');
imagesc( t/1e-3, dop2speed(f,c/fc)/2, pow2db(p), [-30 -28]);
colorbar
ylim([-100 100])
xlim([50 100])
xlabel('Tempo [ms]');
ylabel('Velocidade [m/s]');
title('Mapa Tempo Doppler');

% Range Doppler Response
figure;
rangeDopplerResponse = phased.RangeDopplerResponse(...
    'PropagationSpeed', rp.c,...
    'SampleRate',rp.fs,...
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',1024,...
    'DopplerOutput','Speed',...
    'OperatingFrequency',rp.fc);
filter = getMatchedFilter(radar.Waveform);
plotResponse(...
    rangeDopplerResponse,...
    receivedSignal(:,1:numPulses),...
    filter);
ylim([0 500])
xlim([-100 100])

%Doppler response
figure;
ymf = mf(receivedSignal);
[~,indMax] = max(abs(ymf(:,1)));
N = length(ymf(indMax,:));
fshift = (-N/2:N/2-1)*(rp.prf/N);
Y = fftshift(fft(ymf(indMax,:)));
Y = abs(Y).^2 / N;
speed = dop2speed(fshift, c/fc)/2;
plot(speed,Y);
xlabel('Frequency [Hz]');
ylabel('Power[dB]');
title('Doppler response');
xlim([-75 75])