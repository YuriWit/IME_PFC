classdef HelicopterTarget < AbstractBodyTarget
    % Necessary parameter in p
    % p.c
    % p.fc
    % p.meanBodyRCS
    % p.meanBladeRCS
    % p.radiusVector
    % p.angularVelocityVector
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

            radiusVector = p.radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Blade1 = SpinningPointTarget(bladeParams);
            
            radiusVector = rotationMatrix * radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Blade2 = SpinningPointTarget(bladeParams);

            radiusVector = rotationMatrix * radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Blade3 = SpinningPointTarget(bladeParams);

            radiusVector = rotationMatrix * radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Blade4 = SpinningPointTarget(bladeParams);
        end

        function refrenceUpdate(obj, dt)
            obj.Position = obj.Position + obj.Velocity * dt;
        end

        function forceRefrenceUpdate(obj, newPosition, newVelocity)
            obj.Position = newPosition;
            obj.Velocity = newVelocity;
        end

        function pointsUpdate(obj, dt)
            obj.Body.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Blade1.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Blade2.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Blade3.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Blade4.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Blade1.pointsUpdate(dt);
            obj.Blade2.pointsUpdate(dt);
            obj.Blade3.pointsUpdate(dt);
            obj.Blade4.pointsUpdate(dt);
        end

        function update(obj, dt)
            obj.refrenceUpdate(dt);
            obj.pointsUpdate(dt);
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [obj.Body;obj.Blade1;obj.Blade2;obj.Blade3;obj.Blade4];
        end
    end
end

