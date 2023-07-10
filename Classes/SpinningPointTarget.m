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

        function refrenceUpdate(obj, dt)
            obj.Position = obj.Position + obj.Velocity * dt;
        end

        function forceRefrenceUpdate(obj, newPosition, newVelocity)
            obj.Position = newPosition;
            obj.Velocity = newVelocity;
        end

        function pointsUpdate(obj, dt)
            rvect = obj.RadiusVector;
            wvect = obj.AngularVelocityVector;

            newrvect = rvect + cross(rvect, wvect) * dt;
            newrvect = newrvect .* norm(rvect)/norm(newrvect);

            obj.RadiusVector = newrvect;

            newPosition = obj.Position + newrvect;
            newVelocity = obj.Velocity + cross(newrvect, wvect);

            obj.Point.setPosition(newPosition);
            obj.Point.setVelocity(newVelocity);
        end

        function update(obj, dt)
            obj.refrenceUpdate(dt);
            obj.pointsUpdate(dt);
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [obj.Point];
        end
    end
end

