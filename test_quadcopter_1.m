%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test Case 5
% Stationary four rotor quadcopter with a simgle spining rotor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Start
clear; close all; clc;
addpath("Classes\")
addpath("Scripts\")
rng(2023);

%% Parameters

% Global Params
c = physconst('LightSpeed'); % speed of light (m/s)
fc = 3.7e9; %    central frequency (Hz)
fs = 20e6; % sample rate (Hz)

enviroment = phased.FreeSpace(...
    'PropagationSpeed',c,...
    'OperatingFrequency',fc,...
    'TwoWayPropagation',true,...
    'SampleRate',fs);


% Radar Params (rp for radarParams)
% Simple Radar
rp.c = c;
rp.fc = fc;
rp.fs = fs;
rp.B = 5e6; % sweep bandwidth (Hz)
rp.T = 10e-6; % sweep time (s)
rp.prf = 2e4; % pulse repetition frequency (Hz) (in generated signal)
prf = 125000; % pulse repetition frequency (Hz) (used for time step)
rp.nPulses = 1; % number of pulses (in generated signal)
numPulses = 16384; % number of pulses (in used in simulation)
rp.position = [0;0;0]; % position vector (m)
rp.velocity = [0;0;0]; % velocity vector (m/s)

radar = SimpleRadar(rp);


% Taget Params (tp for targetParams)
% Four Rotor Quadcopter Target
tp.c = c;
tp.fc = fc;
tp.meanBodyRCS = 1; % mean radar cross section for body (m^2)
tp.meanBladeRCS = .5; % mean radar cross section for blades (m^2)
%tp.meanBladeRCS = [.5 .3 .1 .1 .1];
%tp.BladePoints = [.2 .3 .35 .4 .5];
tp.pointsPerBlade = 1; % number of points per blade ()
tp.position = [-250;0;0]; % position vector (m)
tp.velocity = [0;0;0]; % velocity vector (m/s)

% relative position velocity vectors for the rotors (m)
tp.rotor1RelativePosition = .15*[1;1;0];
tp.rotor2RelativePosition = .15*[-1;1;0];
tp.rotor3RelativePosition = .15*[-1;-1;0];
tp.rotor4RelativePosition = .15*[1;-1;0];

% radius vectors (m^3)
r = 0.2548; a1 = 30; a2 = 60; a3 = 90; a4 = 120;
tp.rotor1RadiusVector = r*[cos(a1*pi/180);sin(a1*pi/180);0];
tp.rotor2RadiusVector = r*[cos(a2*pi/180);sin(a2*pi/180);0];
tp.rotor3RadiusVector = r*[cos(a3*pi/180);sin(a3*pi/180);0];
tp.rotor4RadiusVector = r*[cos(a4*pi/180);sin(a4*pi/180);0];

% angular velocity vectors (rad/s)
tp.rotor1AngularVelocityVector = [0;0;1500] *2*pi/60;
tp.rotor2AngularVelocityVector = [0;0;1e-9] *2*pi/60;
tp.rotor3AngularVelocityVector = [0;0;1e-9] *2*pi/60;
tp.rotor4AngularVelocityVector = [0;0;1e-9] *2*pi/60;

target = QuadcopterTarget(tp);


%% Simulate
simulate;

%% Process and Plot

filter = getMatchedFilter(radar.Waveform);
mf = phased.MatchedFilter('Coefficients', filter);
ymf = mf(receivedSignal);
[~,ridx] = max(sum(abs(ymf),2));

% plot short-time Fourier transform
size_fft = 2048;
step = 32;
wave_length = c/fc;
max_doppler_velocity_spread = 51;

window = hamming(size_fft);
time_series = ymf(ridx,:); % needs to have 16384 columns size =  1 16384
[Zxx, f, t] = spectrogram(time_series', window, size_fft - step, size_fft, 125000);
v = f * wave_length / 2;

doppler_velocity_resolution = v(2) - v(1);
Zxx = cat(1,Zxx(end-floor(max_doppler_velocity_spread / doppler_velocity_resolution)+1:end, :), Zxx(1:floor(max_doppler_velocity_spread / doppler_velocity_resolution)+1, :));
Zxx = flip(Zxx, 1);
Zxx = 20 * log10(abs(Zxx));

t = t * 1e3;

figure;
imagesc(t, [-max_doppler_velocity_spread - doppler_velocity_resolution / 2, max_doppler_velocity_spread - doppler_velocity_resolution / 2], Zxx);
set(gca, 'YDir', 'normal');
colormap(jet);
xlabel('Time (ms)');
ylabel('Doppler velocity [m/s]');
title('Time-Doppler map');
colorbar;
clim([10 30]);

%--------------------------------------------------------------------------


% % time doppler map
% figure;
[~,ridx] = max(sum(abs(ymf),2));
% [p,f,t] = pspectrum(ymf(ridx,:),rp.prf,'spectrogram',OverlapPercent=16/16384*100);
% imagesc( t/1e-3, dop2speed(f,c/fc)/2, pow2db(p), [-30 -28]);
% colorbar
% ylim([-100 100])
% xlim([50 100])
% clim([-60 0])
% xlabel('Tempo [ms]');
% ylabel('Velocidade [m/s]');
% title('Mapa Tempo Doppler');
% 
% % Range Doppler Response
% figure;
% rangeDopplerResponse = phased.RangeDopplerResponse(...
%     'PropagationSpeed', rp.c,...
%     'SampleRate',rp.fs,...
%     'DopplerFFTLengthSource','Property',...
%     'DopplerFFTLength',1024,...
%     'DopplerOutput','Speed',...
%     'OperatingFrequency',rp.fc);
% plotResponse(...
%     rangeDopplerResponse,...
%     receivedSignal(:,1:numPulses),...
%     filter);
% ylim([0 500])
% xlim([-100 100])
% 
% %Doppler response
% figure;
% [~,indMax] = max(abs(ymf(:,1)));
% N = length(ymf(indMax,:));
% fshift = (-N/2:N/2-1)*(rp.prf/N);
% Y = fftshift(fft(ymf(indMax,:)));
% Y = abs(Y).^2 / N;
% speed = dop2speed(fshift, c/fc)/2;
% plot(speed,Y);
% xlabel('Frequency [Hz]');
% ylabel('Power[dB]');
% title('Doppler response');
% xlim([-75 75])
























