    function [sinal_banda_base, sinal_filtrado, sinal_decimado] = filtragem(sinal_recepcao, param)
    
        tempo_total = ceil(param.tempo_total/param.intervalo_repeticao)*param.intervalo_repeticao;    
        t_recepcao = 0:1/param.taxa_amostragem:tempo_total-1/param.taxa_amostragem;
        
        sinal_banda_base.sinal = exp(1j*2*pi*param.freq_central*t_recepcao).*sinal_recepcao.sinal;
        sinal_banda_base.freq_range = linspace(-param.taxa_amostragem/2,param.taxa_amostragem/2,length(sinal_banda_base.sinal));
        sinal_banda_base.modulo = fftshift(abs(fft(sinal_banda_base.sinal)));
        sinal_banda_base.fase = fftshift(angle(fft(sinal_banda_base.sinal)));
    
        % Cálculo dos parâmetros do filtro
        Wp = param.largura_banda/(param.taxa_amostragem/2); % Frequência de passagem normalizada
        Ws = 1.1*param.largura_banda/(param.taxa_amostragem/2); % Frequência de rejeição normalizada
        
        % Coeficientes do filtro
        [n, Wn] = cheb2ord(Wp, Ws, param.max_riple, param.atenuacao);
        [b, a] = cheby2(n, param.atenuacao, Wn);
        
        % Filtro Chebyshev tipo 2
        sinal_filtrado.sinal = filter(b, a, sinal_banda_base.sinal);
        sinal_filtrado.freq_range = linspace(-param.taxa_amostragem/2,param.taxa_amostragem/2,length(sinal_filtrado.sinal));
        sinal_filtrado.modulo = fftshift(abs(fft(sinal_filtrado.sinal)));
        sinal_filtrado.fase = fftshift(angle(fft(sinal_filtrado.sinal)));
        
        % Decimação
        len_filt = length(sinal_filtrado.sinal);
        len_dec = ceil(len_filt/param.decimacao);
        sinal_decimado.sinal = zeros(1, len_dec);
        for i = 1:len_dec
           sinal_decimado.sinal(i) = sinal_filtrado.sinal((i-1)*param.decimacao+1); 
        end
        sinal_decimado.freq_range = linspace((-param.taxa_amostragem/param.decimacao)/2,(param.taxa_amostragem/param.decimacao)/2,length(sinal_decimado.sinal));
        sinal_decimado.modulo = fftshift(abs(fft(sinal_decimado.sinal)));
        sinal_decimado.fase = fftshift(angle(fft(sinal_decimado.sinal)));
    
    end