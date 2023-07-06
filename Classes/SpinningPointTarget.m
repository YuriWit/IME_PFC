classdef SpinningPointTarget < AbstractBodyTarget
    % Necessary parameter in p
    % p.c
    % p.fc
    % p.meanRCS
    % p.radiusVector
    % p.angularVelocityVector
    properties
        Point
        Position
        Velocity
        RadiusVector
        AngularVelocityVector
    end
    
    methods
        function obj = SpinningPointTarget(p)
            obj.Position = p.position;
            obj.Velocity = p.velocity;
            obj.RadiusVector = p.radiusVector;
            obj.AngularVelocityVector = p.angularVelocityVector;

            pointParams.c = p.c;
            pointParams.fc = p.fc;
            pointParams.meanRCS = p.meanRCS;
            pointParams.position = [0;0;0];
            pointParams.velocity = [0;0;0];
            obj.Point = PointTarget(pointParams);
        end

        function forceUpdate(obj,newPosition, newVelocity, dt)
            rvect = obj.RadiusVector;
            wvect = obj.AngularVelocityVector;

            newrvect = rvect + cross(rvect, wvect) * dt;
            newrvect = newrvect .* norm(rvect)/norm(newrvect);

            obj.RadiusVector = newrvect;

            newPointPosition = newPosition + newrvect;
            newPointVelocity = newVelocity + cross(newrvect, wvect) * dt;

            obj.Point.update(newPointPosition, newPointVelocity)
        end

        function update(obj, dt)
            obj.Position = obj.Position + obj.Velocity * dt;
            obj.forceUpdate(obj.Position, obj.Velocity, dt);
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [obj.Point];
        end
    end
end

