clear, clc, close all;

%% Definicao dos parametros
% Parâmetros da transmissão
param.duracao_pulso = 10e-6; % s
param.largura_banda = 3e6; % Hz
param.freq_central = 30e6; % Hz
param.freq_tx = 1e9; % Hz
param.taxa_amostragem = 100e6; % Hz

% Parâmetros da recepção
param.posicao = 10e3; % m
param.velocidade = -200; % m/s
param.intervalo_repeticao = .1e-3; % s
%param.tempo_total = 1e-5; % s
param.SNR_dB = 3; % dB

param.max_riple = 1;
param.atenuacao = 30; %dB
param.decimacao = 10;

c = 3e8;
lambda = c / param.freq_tx;

%% Gerar sinal de transmissão
sinal_transmissao = transmissao(param);

figure;
spectrogram(sinal_transmissao.sinal, hamming(128), 120, [], param.taxa_amostragem, 'yaxis');
title('Espectrograma do Sinal de Transmissão');

% %% Definir geometria do corpo
% drone_params.num_de_rotores = 1;
% drone_params.raio_da_helice = 2; % m
% drone_params.pos_dos_rotores = zeros([corpo.num_de_rotores, 2]); % 2d
% drone_params.pos_dos_rotores(1,:) = [0 0]; % m
% drone_params.velocidade_dos_rotores = zeros([corpo.num_de_rotores,1]); % 2d
% drone_params.velocidade_dos_rotores(1,:) = [47]; % rad/s (talzez seja melhor usar RPM)
% drone_params.num_de_pas_por_rotor = 4;
% 
% drone_params.pos_inicial = [3 4 0] % m (x y z) (radar em [0 0 0])
% drone_params.rotacao_inicial = [0 0 0] % Rad (pitch roll yaw) (radar em [0 0 0])
% drone_params.velocidade_inicial = [1 0 0] % m/s (x y z) (radar em [0 0 0])
% drone_params.velocidade_angular_inicial = [0 0 0] % Rad/s (pitch roll yaw) (radar em [0 0 0])

% pra esse codigo, o corpo eh um array de pontos emissores
% [srr x y z vx vy vz]

%emissores = drone2corpo(drone_params, t)
emissor.srr = 1;
emissor.x = 3;
emissor.y = 4;
emissor.z = 0;
emissor.vx = 0;
emissor.vy = 0;
emissor.vz = 0;
emissores = [emissor];


%% Gerar sinais refletidos

% Suponha que N seja o número de sinais recebidos durante uma passagem de antena
N = 32;

% Crie uma matriz vazia para armazenar os sinais recebidos
buffer_sinais_recebidos = [];

% Loop para receber e armazenar os sinais
for i = 1:N
    for j = 1:length(corpo)
        emissor = emissores(j);
        % Gerar sinal de recepção
        sinal_recepcao = recepcao(sinal_transmissao, emissor);
    
        % Gerar replica do sinal transmitido
        replica = sinal_recepcao;
        replica.sinal = [sinal_transmissao.sinal, zeros(1,(length(sinal_recepcao.sinal) - length(sinal_transmissao.sinal)))];
        %replica.sinal = [sinal_transmissao.sinal.*hamming(length(sinal_transmissao.sinal)).', zeros(1,(length(sinal_recepcao.sinal) - length(sinal_transmissao.sinal)))];
    
        % Filtragem do sinal recebido e replica
        [sinal_centralizado, sinal_filtrado, sinal_decimado] = filtragem(sinal_recepcao, param);
        [sinal_centralizado_rep, sinal_filtrado_rep, sinal_decimado_rep] = filtragem(replica, param);
        
        % Fitro casado
        sinal_filtro_casado = filtro_casado(sinal_decimado_rep, sinal_decimado, param);
        
        % Armazenar sinal recebido na matriz
        buffer_sinais_recebidos(:, i) = sinal_filtro_casado.sinal;
        param.posicao = param.posicao + param.velocidade * param.intervalo_repeticao;
    end
end

% % Realizar a integração em tempo lento
% sinal_integrado = sum(buffer_sinais_recebidos, 2) / N;
sinal_integrado = fftshift(fft(buffer_sinais_recebidos.', N, 1),1);

% Agora você pode processar o sinal integrado como desejar
% Por exemplo, plotar o sinal integrado
vetor_distancia = linspace(0,c*param.intervalo_repeticao/2, ceil(param.intervalo_repeticao*(param.taxa_amostragem/param.decimacao)));
vetor_velocidade = linspace(-lambda/(4*param.intervalo_repeticao), lambda/(4*param.intervalo_repeticao), N);

intervalo_recepcao = 2*param.posicao/c;
amostra_delay = ceil(intervalo_recepcao * (param.taxa_amostragem/param.decimacao));
f_doppler = 2*param.velocidade/lambda;
amostra_vel = ceil(N/2 + f_doppler*N*param.intervalo_repeticao/2 );


figure;
spectrogram(sinal_recepcao.sinal, hamming(128), 120, [], param.taxa_amostragem, 'yaxis');
title('Espectrograma do Sinal de Recepcao');

return
figure;
surf(vetor_distancia, vetor_velocidade, 20*log10(abs(sinal_integrado)))
hold on
imagesc(vetor_distancia, vetor_velocidade, 20*log10(abs(sinal_integrado)));
xlabel('Distância');
ylabel('Velocidade');
figure;
subplot(2,1,1);
plot(vetor_velocidade, 20*log10(abs(sinal_integrado(:,amostra_delay))))
xlabel('Velocidade');
ylabel('Amplitude');
subplot(2,1,2);
plot(vetor_distancia, 20*log10(abs(sinal_integrado(amostra_vel,:))))
xlabel('Amplitude');

% Plotar os sinais no tempo e na frequência e seus espectogramas
figure;
subplot(2,2,1);
plot(real(sinal_transmissao.sinal));
title('Sinal de Transmissão no Tempo');
xlabel('Amostras');
ylabel('Amplitude');
subplot(2,2,2);
plot(imag(sinal_transmissao.sinal));
title('Sinal de Transmissão no Tempo');
xlabel('Amostras');
ylabel('Amplitude');
subplot(2,2,3);
plot(real(sinal_recepcao.sinal));
title('Sinal de Recepcao no Tempo');
xlabel('Amostras');
ylabel('Amplitude');
subplot(2,2,4);
plot(imag(sinal_recepcao.sinal));
title('Sinal de Recepcao no Tempo');
xlabel('Amostras');
ylabel('Amplitude');

figure;
subplot(2,2,1);
plot(sinal_transmissao.freq_range, sinal_transmissao.modulo)
title('Espectro do Sinal de Transmissão');
xlabel('Frequência (Hz)');
ylabel('Magnitude');
subplot(2,2,2);
plot(sinal_transmissao.freq_range, sinal_transmissao.fase)
title('Espectro do Sinal de Transmissão');
xlabel('Frequência (Hz)');
ylabel('Fase');
subplot(2,2,3);
plot(sinal_recepcao.freq_range, sinal_recepcao.modulo)
title('Espectro do Sinal de Recepcao');
xlabel('Frequência (Hz)');
ylabel('Magnitude');
subplot(2,2,4);
plot(sinal_recepcao.freq_range, sinal_recepcao.fase);
title('Espectro do Sinal de Recepcao');
xlabel('Frequência (Hz)');
ylabel('Fase');

%figure;
%spectrogram(sinal_transmissao.sinal, [], [], [], param.taxa_amostragem, 'yaxis');
%title('Espectrograma do Sinal de Transmissão');
%figure;
%spectrogram(sinal_integrado, [], [], [], param.taxa_amostragem, 'yaxis');
%title('Espectrograma do Sinal de Recebido');

figure
subplot(3,2,1);
plot(sinal_centralizado.freq_range, sinal_centralizado.modulo)
title('Espectro do Sinal de Banda Base');
xlabel('Frequência (Hz)');
ylabel('Magnitude');
subplot(3,2,2);
plot(sinal_centralizado.freq_range, sinal_centralizado.fase)
title('Espectro do Sinal de Banda Base');
xlabel('Frequência (Hz)');
ylabel('Fase');
subplot(3,2,3);
plot(sinal_filtrado.freq_range, sinal_filtrado.modulo)
title('Espectro do Sinal Filtrado');
xlabel('Frequência (Hz)');
ylabel('Magnitude');
subplot(3,2,4);
plot(sinal_filtrado.freq_range, sinal_filtrado.fase)
title('Espectro do Sinal Filtrado');
xlabel('Frequência (Hz)');
ylabel('Fase');
subplot(3,2,5);
plot(sinal_decimado.freq_range, sinal_decimado.modulo)
title('Espectro do Sinal Decimado');
xlabel('Frequência (Hz)');
ylabel('Magnitude');
subplot(3,2,6);
plot(sinal_decimado.freq_range, sinal_decimado.fase)
title('Espectro do Sinal Decimado');
xlabel('Frequência (Hz)');
ylabel('Fase');

figure
subplot(2,2,1);
plot(real(sinal_filtro_casado.sinal));
title('Sinal do Filtro Casado no Tempo');
xlabel('Amostras');
ylabel('Amplitude');
subplot(2,2,2);
plot(imag(sinal_filtro_casado.sinal));
title('Sinal de Transmissão no Tempo');
xlabel('Amostras');
ylabel('Amplitude');
subplot(2,2,3);
plot(abs(sinal_filtro_casado.sinal));
title('Sinal do Filtro Casado no Tempo');
xlabel('Amostras');
ylabel('Amplitude');
subplot(2,2,4);
plot(20*log10(abs((sinal_filtro_casado.sinal))));
title('Sinal de Transmissão no Tempo');
xlabel('Amostras');
ylabel('Amplitude');
