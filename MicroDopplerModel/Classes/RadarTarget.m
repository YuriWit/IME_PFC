classdef RadarTarget

    properties
        Time
        Position
        Velocity
        % Objects
        Scatters
        Platform
    end
    
    methods
        function obj = RadarTarget(params)
            obj.Time = 0;
            obj.Position = params.targetPosition;
            obj.Velocity = params.targetVelocity;
            obj.Scatters = phased.RadarTarget(...
                'MeanRCS',params.meanRCS,...
                'PropagationSpeed',params.c,...
                'OperatingFrequency',params.sweepCentralFrequency);
            obj.Platform = phased.Platform(...
                'InitialPosition',params.targetPosition,...
                'Velocity',params.targetVelocity);
        end

        function reflectedSignal = getReflectedSignal(obj,signal)
            reflectedSignal = obj.Scatters(signal);
        end

        function [scattersPosition,scattersVelocity] = targetMotion(obj,dt)
            obj.Time = obj.Time+dt;
            [scattersPosition,scattersVelocity] = obj.Platform(dt);
        end
    end
end

