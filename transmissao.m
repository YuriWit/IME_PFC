function sinal_transmissao = transmissao(param)

    % Calculo das frequencias inicial e final
    freq_inicial = param.freq_central - param.largura_banda/2;
    freq_final = param.freq_central + param.largura_banda/2;

    % Sinal de transmissão com modulação linear em frequência (chirp)
    t = 0:1/param.taxa_amostragem:param.duracao_pulso - 1/param.taxa_amostragem;
    sinal_transmissao.sinal = chirp(t, freq_inicial, param.duracao_pulso, freq_final);
    sinal_transmissao.freq_range = linspace(-param.taxa_amostragem/2,param.taxa_amostragem/2,length(sinal_transmissao.sinal));
    sinal_transmissao.modulo = fftshift(abs(fft(sinal_transmissao.sinal)));
    sinal_transmissao.fase = fftshift(angle(fft(sinal_transmissao.sinal)));

end