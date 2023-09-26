% %% Process and Plot
% 
filter = getMatchedFilter(radar.Waveform);
mf = phased.MatchedFilter('Coefficients', filter);
ymf = mf(receivedSignal);
% 
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


%% test plot
size_fft = 2048;
step = 32;
wave_length = c/fc;
max_doppler_velocity_spread = 51;

window = hamming(size_fft);
time_series = ymf(ridx,:); % needs to have 16384 columns size =  1 16384
[Zxx, f, t] = spectrogram(time_series', window, size_fft - step, size_fft, 125000);
v = f * wave_length / 2;

doppler_velocity_resolution = v(2) - v(1);
Zxx = cat(1, Zxx(end-floor(max_doppler_velocity_spread / doppler_velocity_resolution)+1:end, :), Zxx(1:floor(max_doppler_velocity_spread / doppler_velocity_resolution)+1, :));
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






















