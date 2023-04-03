function sinal_filtro_casado = filtro_casado(sinal_transmissao, sinal_recepcao, param)
    h = fliplr(conj(sinal_transmissao.sinal));
    sinal_filtro_casado.sinal = conv(sinal_recepcao.sinal, h);
    sinal_filtro_casado.freq_range = linspace(-param.taxa_amostragem/2,param.taxa_amostragem/2,length(sinal_filtro_casado.sinal));
    sinal_filtro_casado.modulo = fftshift(abs(fft(sinal_filtro_casado.sinal)));
    sinal_filtro_casado.fase = fftshift(angle(fft(sinal_filtro_casado.sinal)));
end