classdef QuadcopterTarget < AbstractBodyTarget
    % Necessary parameter in p
    % p.c
    % p.fc
    % p.meanBodyRCS
    % p.meanBladeRCS
    % p.radiusVector1
    % p.radiusVector2
    % p.radiusVector3
    % p.radiusVector4
    % p.angularVelocityVector1
    % p.angularVelocityVector2
    % p.angularVelocityVector3
    % p.angularVelocityVector4
    properties
        Body
        Prop1Blade1
        Prop1Blade2
        Prop2Blade1
        Prop2Blade2
        Prop3Blade1
        Prop3Blade2
        Prop4Blade1
        Prop4Blade2
        Position
        Velocity
        RadiusVector1
        RadiusVector2
        RadiusVector3
        RadiusVector4
        AngularVelocityVector1
        AngularVelocityVector2
        AngularVelocityVector3
        AngularVelocityVector4
    end
    
    methods
        function obj = QuadcopterTarget(p)
            obj.Position = p.position;
            obj.Velocity = p.velocity;
            obj.RadiusVector1 = p.radiusVector1;
            obj.RadiusVector2 = p.radiusVector2;
            obj.RadiusVector3 = p.radiusVector3;
            obj.RadiusVector4 = p.radiusVector4;
            obj.AngularVelocityVector1 = p.angularVelocityVector1;
            obj.AngularVelocityVector2 = p.angularVelocityVector1;
            obj.AngularVelocityVector3 = p.angularVelocityVector1;
            obj.AngularVelocityVector4 = p.angularVelocityVector1;

            % Body
            bodyParams.c = p.c;
            bodyParams.fc = p.fc;
            bodyParams.meanRCS = p.meanBodyRCS;
            bodyParams.position = [0;0;0];
            bodyParams.velocity = [0;0;0];
            obj.Body = SimpleBodyTarget(bodyParams);

            % Prolepers
            bladeParams.c = p.c;
            bladeParams.fc = p.fc;
            bladeParams.meanRCS = p.meanBladeRCS;
            
            % Proleper 1
            bladeParams.position = [.2;0;0];
            bladeParams.velocity = [0;0;0];
            rotationVector = p.angularVelocityVector1;
            rotationVector = pi * rotationVector/norm(rotationVector);
            rotationMatrix = rotvec2mat3d(rotationVector);
            radiusVector = p.radiusVector1;
            
            bladeParams.angularVelocityVector = p.angularVelocityVector1;
            bladeParams.radiusVector = radiusVector;
            obj.Prop1Blade1 = SpinningPointTarget(bladeParams);
            
            radiusVector = rotationMatrix * radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Prop1Blade2 = SpinningPointTarget(bladeParams);
            
            % Propeler 2
            bladeParams.position = [0;.2;0];
            bladeParams.velocity = [0;0;0];
            rotationVector = p.angularVelocityVector2;
            rotationVector = pi * rotationVector/norm(rotationVector);
            rotationMatrix = rotvec2mat3d(rotationVector);
            radiusVector = p.radiusVector2;

            bladeParams.angularVelocityVector = p.angularVelocityVector2;
            bladeParams.radiusVector = radiusVector;
            obj.Prop2Blade1 = SpinningPointTarget(bladeParams);
            
            radiusVector = rotationMatrix * radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Prop2Blade2 = SpinningPointTarget(bladeParams);
            
            % Propeler 3
            bladeParams.position = [-.2;0;0];
            bladeParams.velocity = [0;0;0];
            rotationVector = p.angularVelocityVector3;
            rotationVector = pi * rotationVector/norm(rotationVector);
            rotationMatrix = rotvec2mat3d(rotationVector);
            radiusVector = p.radiusVector3;

            bladeParams.angularVelocityVector = p.angularVelocityVector3;
            bladeParams.radiusVector = radiusVector;
            obj.Prop3Blade1 = SpinningPointTarget(bladeParams);
            
            radiusVector = rotationMatrix * radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Prop3Blade2 = SpinningPointTarget(bladeParams);
            
            % Propeler 4
            bladeParams.position = [0;-.2;0];
            bladeParams.velocity = [0;0;0];
            rotationVector = p.angularVelocityVector4;
            rotationVector = pi * rotationVector/norm(rotationVector);
            rotationMatrix = rotvec2mat3d(rotationVector);
            radiusVector = p.radiusVector4;

            bladeParams.angularVelocityVector = p.angularVelocityVector4;
            bladeParams.radiusVector = radiusVector;
            obj.Prop4Blade1 = SpinningPointTarget(bladeParams);
            
            radiusVector = rotationMatrix * radiusVector;
            bladeParams.radiusVector = radiusVector;
            obj.Prop4Blade2 = SpinningPointTarget(bladeParams);
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
            obj.Prop1Blade1.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Prop1Blade2.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Prop2Blade1.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Prop2Blade2.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Prop3Blade1.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Prop3Blade2.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Prop4Blade1.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Prop4Blade2.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Body.pointsUpdate();
            obj.Prop1Blade1.pointsUpdate(dt);
            obj.Prop1Blade2.pointsUpdate(dt);
            obj.Prop2Blade1.pointsUpdate(dt);
            obj.Prop2Blade2.pointsUpdate(dt);
            obj.Prop3Blade1.pointsUpdate(dt);
            obj.Prop3Blade2.pointsUpdate(dt);
            obj.Prop4Blade1.pointsUpdate(dt);
            obj.Prop4Blade2.pointsUpdate(dt);
        end

        function update(obj, dt)
            obj.refrenceUpdate(dt);
            obj.pointsUpdate(dt);
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [...
                obj.Body.getPointTargets(), ...
                obj.Prop1Blade1.getPointTargets(), ...
                obj.Prop1Blade2.getPointTargets(), ...
                obj.Prop2Blade1.getPointTargets(), ...
                obj.Prop2Blade2.getPointTargets(), ...
                obj.Prop3Blade1.getPointTargets(), ...
                obj.Prop3Blade2.getPointTargets(), ...
                obj.Prop4Blade1.getPointTargets(), ...
                obj.Prop4Blade2.getPointTargets()];
        end
    end
end

