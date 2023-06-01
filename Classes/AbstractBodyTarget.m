classdef (Abstract) AbstractBodyTarget < handle

    properties  
        Position
        Velocity
    end

    methods
        function obj = AbstractBodyTarget(p)
            obj.Position = p.targetPosition;
            obj.Velocity = p.targetVelocity;
        end
    end
    
    methods (Abstract)
        update(obj,dt)
        pointTargets = getPointTargets(obj)
    end

end

