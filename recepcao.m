function sinal_recebido = recepcao(sinal_transmissao, param)
    c = 3e8;
    lambda = c / param.freq_central;

    % Sinal de recepção
    intervalo_recepcao = 2*param.posicao/c;
    tempo_total = ceil(param.tempo_total/param.intervalo_repeticao)*param.intervalo_repeticao;
    t_recepcao = 0:1/param.taxa_amostragem:tempo_total-1/param.taxa_amostragem;

    delay = ceil(intervalo_recepcao / t_recepcao(2));

    sinal_recepcao = zeros(size(t_recepcao));
    sinal_recepcao(1:length(sinal_transmissao.sinal)) = sinal_transmissao.sinal;
    sinal_recepcao = [zeros(1,delay) sinal_recepcao(1:end-delay)];

    % Efeito do Doppler no sinal de recepção e ruído térmico
    f_doppler = 2*param.velocidade*param.freq_central/c;
    Potencia_sinal = norm(sinal_recepcao)^2/length(sinal_recepcao);
    Potencia_ruido = Potencia_sinal/(10^(param.SNR_dB/10));
    ruido = sqrt(Potencia_ruido/2)*(randn(size(sinal_recepcao)) + 1j*randn(size(sinal_recepcao)));
    
    sinal_recebido.sinal = real(exp(1j*4*pi*param.posicao/lambda) .* exp(1j*2*pi*f_doppler*t_recepcao) .* sinal_recepcao + ruido);
    sinal_recebido.freq_range = linspace(-param.taxa_amostragem/2,param.taxa_amostragem/2,length(sinal_recebido.sinal));
    sinal_recebido.modulo = fftshift(abs(fft(sinal_recebido.sinal)));
    sinal_recebido.fase = fftshift(angle(fft(sinal_recebido.sinal)));
    
end
