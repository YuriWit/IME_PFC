function sinal_filtro_casado = filtro_casado(replica, sinal_recepcao, param)
    h = fliplr(conj(replica.sinal));
    %sinal_filtro_casado.sinal = conv(sinal_recepcao.sinal, h,'same');
    sinal_filtro_casado.sinal = ifft(fft(sinal_recepcao.sinal).*conj(fft(replica.sinal)));
    sinal_filtro_casado.freq_range = linspace(-param.taxa_amostragem/2,param.taxa_amostragem/2,length(sinal_filtro_casado.sinal));
    sinal_filtro_casado.modulo = fftshift(abs(fft(sinal_filtro_casado.sinal)));
    sinal_filtro_casado.fase = fftshift(angle(fft(sinal_filtro_casado.sinal)));
end