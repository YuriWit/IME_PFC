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
rp.prf = 2e4; % pulse repetition frequency (Hz)
rp.nPulses = 1; % number of pulses
rp.position = [0;0;0]; % position vector (m)
rp.velocity = [0;0;0]; % velocity vector (m/s)

% Helicopter Target two blades main rotor no tail totor
% tp for targetParams
tp.c = c;
tp.fc = fc;
tp.meanBodyRCS = 1; % mean body cross section (m^2)
tp.meanRCS = 10; % mean body cross section (m^2)
tp.meanBladeRCS = .1; % mean blase cross section (m^2)

tp.position = [-1000;0;0]; % position vector (m)
tp.velocity = [100;0;0]; % velocity vector (m/s)

tp.rotor1RelativePosition = .1*[1;1;0]; % relative position velocity vector (m/s)
tp.rotor2RelativePosition = .1*[-1;1;0]; % relative position velocity vector (m/s)
tp.rotor3RelativePosition = .1*[-1;-1;0]; % relative position velocity vector (m/s)
tp.rotor4RelativePosition = .1*[1;-1;0]; % relative position velocity vector (m/s)

r = .0331/2;
tp.rotor1RadiusVector = r*[cos(30*pi/180);sin(30*pi/180);0]; % radius vector (m^3)
tp.rotor2RadiusVector = r*[cos(22*pi/180);sin(22*pi/180);0]; % radius vector (m^3)
tp.rotor3RadiusVector = r*[cos(56*pi/180);sin(56*pi/180);0]; % radius vector (m^3)
tp.rotor4RadiusVector = r*[cos(12*pi/180);sin(12*pi/180);0]; % radius vector (m^3)

tp.rotor1AngularVelocityVector = [0;0;1e-10] *2*pi/60; % angular velocity vector (rad/s)
tp.rotor2AngularVelocityVector = [0;0;1e-10] *2*pi/60; % angular velocity vector (rad/s)
tp.rotor3AngularVelocityVector = [0;0;1e-10] *2*pi/60; % angular velocity vector (rad/s)
tp.rotor4AngularVelocityVector = [0;0;1e-10] *2*pi/60; % angular velocity vector (rad/s)

%% Initiate Objects
radar = SimpleRadar(rp);
target = SimpleBodyTarget(tp);
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
rangeDopplerResponse = phased.RangeDopplerResponse(...
    'PropagationSpeed', rp.c,...
    'SampleRate',rp.fs,...
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',128,...
    'DopplerOutput','Speed',...
    'OperatingFrequency',rp.fc);

filter = getMatchedFilter(radar.Waveform);
mf = phased.MatchedFilter('Coefficients',filter);

%% Plots
filter = getMatchedFilter(radar.Waveform);
mf = phased.MatchedFilter('Coefficients', filter);
ymf = mf(receivedSignal);


% transmited signal
figure;
t1 = linspace(0,1/rp.prf,length(transmittedSignal(:,1)));
plot(t1,real(transmittedSignal(:,1)));
title('transmited signal');

% transmited signal matched filter output
figure;
tymf = mf(transmittedSignal);
t2 = linspace(0,rp.T,length(tymf(:,1)));
plot(t2,abs(tymf(:,1)).^2);
title('transmited signal matched filter output');

% % transmited signal spectrum
% figure;
% spectrogram(radar.Waveform(),64,60,1024,fs,'yaxis','centered');
% spectrogram(s,'yaxis')

% % waveform spectrum
% figure;
% Y1 = fftshift(fft(radar.Waveform()));
% f1 = linspace(-fs/2,fs/2,length(Y1))/1e6;
% plot(f1,abs(Y1).^2);
% xlabel('X(f)');
% ylabel('Frequency [MHz]');
% title('Transmited waveform spectrum');

% received signal
figure;
t2 = linspace(0,1/rp.prf,length(receivedSignal(:,1)));
plot(t2,real(receivedSignal(:,1)).^2);
title('received signal');

% received signal matched filter output
figure;
t2 = linspace(0,rp.T,length(ymf(:,1)));
plot(t2,abs(ymf(:,1)).^2);
title('received signal matched filter output');

% slow time response
figure;
t = (1:1:length(receivedSignal(:,1)))/fs;
FFTreceivedSignal = fft(receivedSignal');%FFT of each line
maxFFTreceivedSignal = max(abs(FFTreceivedSignal));
plot(t, 10*log10(abs(maxFFTreceivedSignal).^2));
xlabel('Time [ms]');
ylabel('h(t)');
title('Slow time response');



return
% fast time response
figure;
t = (1:1:length(ymf(:,1)))/fs;
plot(t, 10*log10(abs(ymf(:,1)).^2));
xlabel('Time [s]');
ylabel('CCF in dB');
title('Fast time response');

% slow time response
figure;
FFTymf = fft(ymf');%FFT of each column
maxFFTymf = max(abs(FFTymf));
plot(t, 10*log10(abs(maxFFTymf).^2));
xlabel('Time [ms]');
ylabel('h(t)');
title('Slow time response');

% spectrum
figure;
[~,indMax] = max(abs(ymf(:,1)));
plot(linspace(-fs/2,fs/2,rp.T*fs)/1e6,10*log10(abs(fftshift(fft(receivedSignal(indMax-rp.T*fs+1:indMax,1))))))
xlabel('Frequency [MHz]');
ylabel('Power[dBFS]');
title('Spectrum');

%Doppler response
figure;
plot(10*log10(abs(fftshift(fft(receivedSignal(1:indMax,1))))))
xlabel('Frequency [Hz]');
ylabel('Power[dB]');
title('Doppler response');

% range doppler response
figure;
rangeDopplerResponse = phased.RangeDopplerResponse(...
    'RangeMethod','Matched filter',...
    'PropagationSpeed',rp.c,...
    'SampleRate',rp.fs,...
    'PRFSource','Property',...
    'PRF',rp.prf,...
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',1024,...
    'DopplerOutput','Frequency',...
    'OperatingFrequency',rp.fc);
plotResponse(...
    rangeDopplerResponse,...
    receivedSignal(:,1:numPulses),...
    filter);
ylim([0 1000])
xlim([-100 100])

% time doppler response
figure;
doppler_response = fftshift(fft(maxFFTymf));
f3 = linspace(-rp.prf / 2, rp.prf / 2, numel(maxFFTymf));
v3 = f3 * c/rp.fs / 2;
doppler_velocity_resolution = v3(2) - v3(1);
plot(f3, 20 * log10(abs(doppler_response)));
% scope = phased.DTIScope(...
%     'IQDataInput', true,...,
%     'DopplerOffset',-rp.prf/4,...
%     'DopplerOutput','Frequency',...
%     'OperatingFrequency',rp.fc,...
%     'DopplerFFTLength',1024);
% scope(ymf);


    %'DopplerOffset',-220,...
