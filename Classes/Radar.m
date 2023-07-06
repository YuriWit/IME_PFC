classdef Radar < handle

    properties
        % Dynamic
        Time
        Position
        Velocity

        % Objects
        Waveform
        Transmitter
        Receiver
        Radiator
        Collector
    end
    
    methods
        function obj = Radar(p) % p = params
            obj.Time = 0;
            obj.Position = p.position;
            obj.Velocity = [0;0;0];
            obj.Waveform = p.Waveform;
            obj.Transmitter = p.Transmitter;
            obj.Receiver = p.Receiver;
            obj.Radiator = p.Radiator;
            obj.Collector = p.Collector;
        end
        
        function transmittedSignal = getTransmittedSignal(obj,transmissionAngle)
            pulseWaveformSamples = obj.Waveform();
            transmissionSignal = obj.Transmitter(pulseWaveformSamples);
            transmittedSignal = obj.Radiator(transmissionSignal,transmissionAngle);
        end

        function receivedSignal = receiveReflectedSignal(obj,signal,receptionAngle)
            collectedSignal = obj.Collector(signal,receptionAngle);
            receivedSignal = obj.Receiver(collectedSignal);
        end

        function update(obj,dt)
            obj.Time = obj.Time + dt;
        end

    end
end

