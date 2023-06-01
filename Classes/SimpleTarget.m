classdef SimpleTarget < handle
    
    properties
        Position
        Velocity

        Point
    end
    
    methods
        function obj = SimpleTarget(p)
            bp.c = p.c;
            bp.fc = p.fc;
            bp.meanRCS = p.meanRCS;
            bp.position = p.targetPosition;
            bp.velocity = p.targetVelocity;
            obj.Point = PointTarget(bp);
            obj.Position = p.targetPosition;
            obj.Velocity = p.targetVelocity;
        end

        function update(obj,dt)
            obj.Position = obj.Position + obj.Velocity * dt;
            obj.Point.Position = obj.Position;
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [obj.Point];
        end
    end
end

