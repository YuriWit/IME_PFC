classdef QuadcopterTarget < AbstractBodyTarget

    properties
        Position
        Velocity
        Rotor1Position
        Rotor2Position
        Rotor3Position
        Rotor4Position

        Body
        Rotor1
        Rotor2
        Rotor3
        Rotor4
    end
    
    methods
        function obj = QuadcopterTarget(p)
            obj.Position = p.position;
            obj.Velocity = p.velocity;
            obj.Rotor1Position = p.rotor1RelativePosition;
            obj.Rotor2Position = p.rotor2RelativePosition;
            obj.Rotor3Position = p.rotor3RelativePosition;
            obj.Rotor4Position = p.rotor4RelativePosition;

            % Body
            bodyParams.c = p.c;
            bodyParams.fc = p.fc;
            bodyParams.meanRCS = p.meanBodyRCS;
            bodyParams.position = [0;0;0];
            bodyParams.velocity = [0;0;0];
            obj.Body = SimpleBodyTarget(bodyParams);

            % Rotors
            rotorParams.c = p.c;
            rotorParams.fc = p.fc;
            rotorParams.meanBladeRCS = p.meanBladeRCS;
            rotorParams.bladePoints = p.bladePoints;
            rotorParams.position = [0;0;0];
            rotorParams.velocity = [0;0;0];
    
            % Rotor 1
            rotorParams.radiusVector = p.rotor1RadiusVector;
            rotorParams.angularVelocityVector = p.rotor1AngularVelocityVector;
            obj.Rotor1 = TwoBladeRotorTarget(rotorParams);

            % Rotor 2
            rotorParams.radiusVector = p.rotor2RadiusVector;
            rotorParams.angularVelocityVector = p.rotor2AngularVelocityVector;
            obj.Rotor2 = TwoBladeRotorTarget(rotorParams);

            % Rotor 3
            rotorParams.radiusVector = p.rotor3RadiusVector;
            rotorParams.angularVelocityVector = p.rotor3AngularVelocityVector;
            obj.Rotor3 = TwoBladeRotorTarget(rotorParams);

            % Rotor 4
            rotorParams.radiusVector = p.rotor4RadiusVector;
            rotorParams.angularVelocityVector = p.rotor4AngularVelocityVector;
            obj.Rotor4 = TwoBladeRotorTarget(rotorParams);
        end

        function refrenceUpdate(obj, dt)
            obj.Position = obj.Position + obj.Velocity * dt;
            p = obj.Position;
            v = obj.Velocity;
            obj.Body.forceRefrenceUpdate(p, v);
            obj.Rotor1.forceRefrenceUpdate(p + obj.Rotor1Position, v);
            obj.Rotor2.forceRefrenceUpdate(p + obj.Rotor2Position, v);
            obj.Rotor3.forceRefrenceUpdate(p + obj.Rotor3Position, v);
            obj.Rotor4.forceRefrenceUpdate(p + obj.Rotor4Position, v);
        end

        function forceRefrenceUpdate(obj, newPosition, newVelocity)
            obj.Position = newPosition;
            obj.Velocity = newVelocity;
            p = obj.Position;
            v = obj.Velocity;
            obj.Body.forceRefrenceUpdate(p, v);
            obj.Rotor1.forceRefrenceUpdate(p + obj.Rotor1Position, v);
            obj.Rotor2.forceRefrenceUpdate(p + obj.Rotor2Position, v);
            obj.Rotor3.forceRefrenceUpdate(p + obj.Rotor3Position, v);
            obj.Rotor4.forceRefrenceUpdate(p + obj.Rotor4Position, v);
        end

        function pointsUpdate(obj, dt)
            obj.Body.pointsUpdate();
            obj.Rotor1.pointsUpdate(dt);
            obj.Rotor2.pointsUpdate(dt);
            obj.Rotor3.pointsUpdate(dt);
            obj.Rotor4.pointsUpdate(dt);
        end

        function update(obj, dt)
            obj.refrenceUpdate(dt);
            obj.pointsUpdate(dt);
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [...
                obj.Body.getPointTargets(), ...
                obj.Rotor1.getPointTargets(), ...
                obj.Rotor2.getPointTargets(), ...
                obj.Rotor3.getPointTargets(), ...
                obj.Rotor4.getPointTargets()];
        end
    end
end

