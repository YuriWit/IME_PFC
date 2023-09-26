classdef TwoBladeRotorTarget < AbstractBodyTarget
    %ROTORTARGET Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Position
        Velocity

        Blade1
        Blade2
    end
    
    methods
        function obj = TwoBladeRotorTarget(p)
            %ROTORTARGET Construct an instance of this class
            %   Detailed explanation goes here
            obj.Position = p.position;
            obj.Velocity = p.velocity;

            % Blades
            bladeParams.c = p.c;
            bladeParams.fc = p.fc;
            bladeParams.meanRCS = p.meanBladeRCS;
            bladeParams.position = [0;0;0];
            bladeParams.velocity = [0;0;0];
            bladeParams.angularVelocityVector = p.angularVelocityVector;
            bladeParams.pointsPerBlade = p.pointsPerBlade;

            % Blade 1
            bladeParams.radiusVector = p.radiusVector;
            obj.Blade1 = Blade(bladeParams);

            % Blade 2
            bladeParams.radiusVector = -p.radiusVector;
            obj.Blade2 = Blade(bladeParams);
        end

        function refrenceUpdate(obj, dt)
            obj.Position = obj.Position + obj.Velocity * dt;
            obj.Blade1.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Blade2.forceRefrenceUpdate(obj.Position, obj.Velocity);
        end

        function forceRefrenceUpdate(obj, newPosition, newVelocity)
            obj.Position = newPosition;
            obj.Velocity = newVelocity;
            obj.Blade1.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Blade2.forceRefrenceUpdate(obj.Position, obj.Velocity);
        end

        function pointsUpdate(obj, dt)
            obj.Blade1.pointsUpdate(dt);
            obj.Blade2.pointsUpdate(dt);
        end

        function update(obj, dt)
            obj.refrenceUpdate(dt);
            obj.pointsUpdate(dt);
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [...
                obj.Blade1.getPointTargets(), ...
                obj.Blade2.getPointTargets()];
        end
    end
end

