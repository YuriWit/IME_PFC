classdef Blade < AbstractBodyTarget

    properties
        Length
        Angle
        AngularVelocity

        PointTargets
    end
    
    methods
        function obj = Blade(p)
            obj@AbstractBodyTarget(p);

            obj.Length = p.length;
            obj.Angle = p.angle;
            obj.AngularVelocity = p.angularVelocity;

            op.position = [0;0;0];
            op.velocity = [0;0;0];
            op.meanRCS = p.meanRCS;
            op.c = p.c;
            op.fc = p.fc;
            
            pointTargets(p.numberOfPointTargets) = PointTarget(op);
            obj.PointTargets = pointTargets;
            obj.update(0);
        end

        function update(obj,dt)
            update@AbstractBodyTarget(obj,dt);

            obj.Angle = obj.Angle + dt * obj.AngularVelocity;
            tipPosition = [ cos(obj.Angle)*obj.Length;
                            sin(obj.Angle)*obj.Length;
                            0];
            tipVelocity = [-sin(obj.Angle)*obj.Length*obj.AngularVelocity;
                            cos(obj.Angle)*obj.Length*obj.AngularVelocity;
                            0];
            numberOfPointTargets = length(obj.PointTargets);
            for i=1:numberOfPointTargets
                obj.PointTargets(i).Position = tipPosition * i / numberOfPointTargets;
                obj.PointTargets(i).Velocity = tipVelocity * obj.Length * i / numberOfPointTargets;
            end
        end

        function pointTargets = getPointTargets(obj)
            pointTargets = obj.PointTargets;
        end

    end
end

