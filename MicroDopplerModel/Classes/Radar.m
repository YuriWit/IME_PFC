classdef Radar

    properties
        % Radar
        Time
        Position
        Velocity

        % Objects
        WaveForm
        Transmitter
        Receiver
        TransmissionAntena
        CollectionAntena
    end
    
    methods
        function obj = Radar(params)
            obj.Time = 0;
            obj.Position = params.radarPosition;
            obj.Velocity = params.radarVelocity;
            
            start_f = params.sweepCentralFrequency-params.sweepBandwidth/2;
            end_f = params.sweepCentralFrequency+params.sweepBandwidth/2;
            frequencyModulation = [start_f end_f];
            
            obj.WaveForm = phased.CustomFMWaveform(...
                'SampleRate',params.sampleRate,...
                'PulseWidth',params.sweepTime,...
                'FrequencyModulation',frequencyModulation,...
                'NumPulses',params.numberOfPulses);

            obj.Transmitter = phased.Transmitter;
            obj.Receiver = phased.ReceiverPreamp;

            uniformRectangularArray = phased.URA(...
                'Size',4,...
                'ElementSpacing',params.c/params.sweepCentralFrequency/2);

            obj.TransmissionAntena = phased.Radiator(...
                'Sensor',uniformRectangularArray,...
                'PropagationSpeed',params.c,...
                'OperatingFrequency',params.sweepCentralFrequency);

            obj.CollectionAntena = phased.Collector(...
                'Sensor',uniformRectangularArray,...
                'PropagationSpeed',params.c,...
                'OperatingFrequency',params.sweepCentralFrequency);
        end
        
        function transmittedSignal = getTransmittedSignal(obj,angle)
            pulseWaveformSamples = obj.WaveForm();
            transmissionSignal = obj.Transmitter(pulseWaveformSamples);
            transmittedSignal = obj.TransmissionAntena(transmissionSignal,angle);
        end

        function receivedSignal = receiveReflectedSignal(obj,signal,arrivingAngle)
            collectedSignal = obj.CollectionAntena(signal,arrivingAngle);
            receivedSignal = obj.Receiver(collectedSignal);
        end

        function [radarPosition,radarVelocity] = radarMotion(obj,dt)
            obj.Time = obj.Time+dt;
            [radarPosition radarVelocity] = [obj.Position obj.Velocity];
        end
    end
end

