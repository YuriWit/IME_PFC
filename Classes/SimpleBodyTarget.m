classdef SimpleBodyTarget < AbstractBodyTarget
    % Necessary parameter in p
    % p.c
    % p.fc
    % p.meanRCS
    % p.position
    % p.velocity
    properties
        Point
        Position
        Velocity
    end
    
    methods

        function obj = SimpleBodyTarget(p)
            obj.Position = p.position;
            obj.Velocity = p.velocity;

            pointParams.c = p.c;
            pointParams.fc = p.fc;
            pointParams.meanRCS = p.meanRCS;
            pointParams.position = [0;0;0];
            pointParams.velocity = [0;0;0];
            obj.Point = PointTarget(pointParams);
        end

        function forceUpdate(obj,newPosition, newVelocity, dt)
            obj.Point.update(newPosition, newVelocity);
        end

        function update(obj, dt)
            obj.Position = obj.Position + obj.Velocity * dt;
            obj.Point.update(obj.Position, obj.Velocity);
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [obj.Point];
        end

    end

end

