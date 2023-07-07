classdef PointTarget < handle
    % Parameters
    % p.c
    % p.fc
    % p.meanRCS
    % p.position
    % p.velocity
    properties
        Position
        Velocity
        Target
    end
    
    methods

        function obj = PointTarget(p)
            obj.Target = phased.RadarTarget(...
                'PropagationSpeed',p.c,...
                'OperatingFrequency',p.fc,...
                'MeanRCS',p.meanRCS);
            obj.Position = p.position;
            obj.Velocity = p.velocity;
        end

        function setPosition(obj, newPosition)
            obj.Position = newPosition;
        end

        function setVelocity(obj, newVelocity)
            obj.Velocity = newVelocity;
        end

        function timeStep(obj, dt)
            obj.Position = obj.Position + obj.Velocity * dt;
        end

        function reflectedSignal = getReflectedSignal(obj,signal)
            reflectedSignal = obj.Target(signal);
        end

    end

end

