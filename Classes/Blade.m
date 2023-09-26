classdef Blade < AbstractBodyTarget

    properties
        Position
        Velocity
        
        PointTargets
    end
    
    methods
        function obj = Blade(p)
            obj.Position = p.position;
            obj.Velocity = p.velocity;

            op.c = p.c;
            op.fc = p.fc;
            op.meanRCS = p.meanRCS;
            op.position = [0;0;0];
            op.velocity = [0;0;0];
            op.angularVelocityVector = p.angularVelocityVector;
            op.radiusVector = p.radiusVector;
            
            pointTargets(p.pointsPerBlade) = SpinningPointTarget(op);

            for i=1:p.pointsPerBlade
                rv = p.radiusVector * i / (p.pointsPerBlade + 1);
                op.radiusVector = rv;
                pointTargets(i) = SpinningPointTarget(op);
            end

            obj.PointTargets = pointTargets;
        end

        function refrenceUpdate(obj, dt)
            obj.Position = obj.Position + obj.Velocity * dt;
            pointsPerBlade = length(obj.PointTargets);
            for i=1:pointsPerBlade
                obj.PointTargets(i).forceRefrenceUpdate(obj.Position, obj.Velocity);
            end
        end

        function forceRefrenceUpdate(obj, newPosition, newVelocity)
            obj.Position = newPosition;
            obj.Velocity = newVelocity;
            pointsPerBlade = length(obj.PointTargets);
            for i=1:pointsPerBlade
                obj.PointTargets(i).forceRefrenceUpdate(obj.Position, obj.Velocity);
            end
        end

        function pointsUpdate(obj, dt)
            pointsPerBlade = length(obj.PointTargets);
            for i=1:pointsPerBlade
                obj.PointTargets(i).pointsUpdate(dt);
            end
        end

        function update(obj,dt)
            obj.refrenceUpdate(dt);
            obj.pointsUpdate(dt);
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [obj.PointTargets.getPointTargets()];
        end
    end
end

