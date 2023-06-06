classdef SimpleTarget < AbstractBodyTarget

    properties
        Point1
        Point2
    end
    
    methods

        function obj = SimpleTarget(p)
            obj@AbstractBodyTarget(p);

            bp.c = p.c;
            bp.fc = p.fc;
            bp.meanRCS = p.meanRCS;

            bp.position = p.p1.Position;
            bp.velocity = p.p1.Velocity;
            obj.Point1 = PointTarget(bp);

            bp.position = p.p2.Position;
            bp.velocity = p.p2.Velocity;
            obj.Point2 = PointTarget(bp);
        end

        function update(obj,dt)
            % body
            update@AbstractBodyTarget.update(dt);

            % points
            obj.Point1.Position = obj.Point1.Position + ...
                obj.Point1.Velocity * dt;

            obj.Point2.Position = obj.Point2.Position + ...
                obj.Point2.Velocity * dt;
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = [obj.Point1 obj.Point2];
        end

    end

end

