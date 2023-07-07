classdef HelicopterTarget
    % Necessary parameter in p
    % p.c
    % p.fc
    % p.meanRCS
    % p.radiusVector
    % p.angularVelocityVector
    % p.meanBodyRCS
    % p.meanBladeRCS
    properties
        Body
        Blade1
        Blade2
        Blade3
        Blade4
        Position
        Velocity
        RadiusVector
        AngularVelocityVector
    end
    
    methods
        function obj = HelicopterTarget(p)
            obj.Position = p.position;
            obj.Velocity = p.velocity;
            obj.RadiusVector = p.radiusVector;
            obj.AngularVelocityVector = p.angularVelocityVector;

            % Body
            bodyParams.c = p.c;
            bodyParams.fc = p.fc;
            bodyParams.meanRCS = p.meanBodyRCS;
            bodyParams.position = [0;0;0];
            bodyParams.velocity = [0;0;0];
            obj.Body = PointTarget(bodyParams);

            % Blades
            bladeParams.c = p.c;
            bladeParams.fc = p.fc;
            bladeParams.meanRCS = p.meanBladeRCS;
            bladeParams.position = [0;0;0];
            bladeParams.velocity = [0;0;0];
            bladeParams.angularVelocityVector = p.angularVelocityVector;

            rotationVector = p.angularVelocityVector;
            rotationVector = pi/2 * rotationVector/norm(rotationVector);
            rotationMatrix = rotvec2mat3d(rotationVector);

            radiusVector = p.RadiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Blade1 = SpinningPointTarget(bladeParams);
            
            radiusVector = rotationMatrix * radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Blade2 = PointTarget(bladeParams);

            radiusVector = rotationMatrix * radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Blade3 = PointTarget(bladeParams);

            radiusVector = rotationMatrix * radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Blade4 = PointTarget(bladeParams);
        end

        function forceUpdate(obj,newPosition, newVelocity, dt)
            obj.Position = newPosition;
            obj.Velocity = newVelocity;
            

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

