classdef SimpleRadar < Radar
    properties
        % no aditional properties
    end
    
    methods
        % p     = parameters object
        % fc    = central frequency of chirp signal
        % B     = Bandwitdt
        % T     = chirp signal width
        function obj = SimpleRadar(p)
            % chirp waveform creation
            start_f = p.fc - p.B/2;
            end_f = p.fc + p.B/2;
            p.Waveform = phased.CustomFMWaveform(...
                'SampleRate',p.fs,...
                'PulseWidth',p.T,...
                'FrequencyModulation',[start_f, end_f],...
                'NumPulses',p.nPulses,...
                'PRF',p.prf);
            
            % simple default Transmitter
            p.Transmitter = phased.Transmitter;

            % simple default Receiver
            p.Receiver = phased.ReceiverPreamp;

            % simple sensor for Radiator and Collector
            sensor = phased.IsotropicAntennaElement(...
                'FrequencyRange',[1e9 10e9]);

            % simple TransmissionAntena
            p.Radiator = phased.Radiator(...
                'Sensor',sensor,...
                'PropagationSpeed',p.c,...
                'OperatingFrequency',p.fc);

            % simple CollectionAntena
            p.Collector = phased.Collector(...
                'Sensor',sensor,...
                'PropagationSpeed',p.c,...
                'OperatingFrequency',p.fc);

            obj@Radar(p);
        end
    end
end

