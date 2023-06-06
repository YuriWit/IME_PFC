classdef HelicopterTarget < AbstractBodyTarget
    
    properties
        Body
        Blades
    end
    
    methods
        function obj = HelicopterTarget(p)
            obj@AbstractBodyTarget(p);
            
            % body
            bp.c = p.c;
            bp.fc = p.fc;
            bp.meanRCS = p.bodyRCS;
            bp.Position = p.targetPosition;
            bp.Velocity = p.targetVelocity;
            obj.Body = PointTarget(bp);
            
            % blades
            bp.meanRCS = p.bladeRCS;
            bp.length = p.bladeLenth;
            bp.angularVelocity = p.angularVelocity;
            bp.numberOfPointTargets = p.numberOfPointTargetsPerBlade;
            blades(p.numberOfBlades) = Blade(bp);
            for i=1:p.numberOfBlades
                bp.angle = p.angle + i * 2 * pi / p.nuberOfBlades;
                blades(i) = Blade(bp);
            end
            obj.Blades = blades;
            obj.update(0);
        end

        function update(obj,dt)
            update@AbstractBodyTarget(obj,dt);

            numberOfBlades = length(obj.Blades);
            for i=1:numberOfBlades
                obj.Blades(i).update(dt);
            end
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [obj.Body obj.Blades];
        end
    end
end

