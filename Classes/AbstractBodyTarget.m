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

        function update(obj,dt)
            obj.Position = obj.Position + dt * obj.Velocity;
        end
    end
    
    methods (Abstract)
        pointTargets = getPointTargets(obj)
    end

end

