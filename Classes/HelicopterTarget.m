classdef HelicopterTarget < AbstractBodyTarget

    properties
        Position
        Velocity

        Body
        Rotor
    end
    
    methods
        function obj = HelicopterTarget(p)
            obj.Position = p.position;
            obj.Velocity = p.velocity;

            % Body
            bodyParams.c = p.c;
            bodyParams.fc = p.fc;
            bodyParams.meanRCS = p.meanBodyRCS;
            bodyParams.position = [0;0;0];
            bodyParams.velocity = [0;0;0];
            obj.Body = SimpleBodyTarget(bodyParams);

            % Rotor
            rotorParams.c = p.c;
            rotorParams.fc = p.fc;
            rotorParams.meanBladeRCS = p.meanBladeRCS;
            rotorParams.position = [0;0;0];
            rotorParams.velocity = [0;0;0];
            rotorParams.radiusVector = p.radiusVector;
            rotorParams.angularVelocityVector = p.angularVelocityVector;
            rotorParams.pointsPerBlade = p.pointsPerBlade;
            obj.Rotor = TwoBladeRotorTarget(rotorParams);
        end

        function refrenceUpdate(obj, dt)
            obj.Position = obj.Position + obj.Velocity * dt;
            obj.Body.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Rotor.forceRefrenceUpdate(obj.Position, obj.Velocity);
        end

        function forceRefrenceUpdate(obj, newPosition, newVelocity)
            obj.Position = newPosition;
            obj.Velocity = newVelocity;
            obj.Body.forceRefrenceUpdate(obj.Position, obj.Velocity);
            obj.Rotor.forceRefrenceUpdate(obj.Position, obj.Velocity);
        end

        function pointsUpdate(obj, dt)
            obj.Body.pointsUpdate();
            obj.Rotor.pointsUpdate(dt);
        end

        function update(obj, dt)
            obj.refrenceUpdate(dt);
            obj.pointsUpdate(dt);
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [...
                obj.Body.getPointTargets(), ...
                obj.Rotor.getPointTargets()];
        end
    end
end

