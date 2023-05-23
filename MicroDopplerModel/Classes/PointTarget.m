classdef PointTarget < handle

    properties
        Position
        Velocity
        Target
    end
    
    methods

        function obj = PointTarget(p)
            obj.Position = p.targetPosition;
            obj.Velocity = p.targetVelocity;
            obj.Target = phased.RadarTarget(...
                'MeanRCS',p.meanRCS,...
                'PropagationSpeed',p.c,...
                'OperatingFrequency',p.fc);
        end

        function reflectedSignal = getReflectedSignal(obj,signal)
            reflectedSignal = obj.Target(signal);
        end

    end
end

