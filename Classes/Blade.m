classdef Blade

    properties
        AngularPosition
        AngularVelocity
        Length

        PointTargets
    end
    
    methods
        function obj = Blade(p)
            obj.AngularPosition = p.angularPosition;
            obj.AngularVelocity = p.angularVelocity;
            obj.Length = p.length;
            
            for i=1:p.numberOfPointTargets
                op.Position = p.angularPosition * p.length * i / numberOfPointTargets;
                op.Velocity = 
            end

            obj.NumberOfPointTargets = p.numberOfPointTargets;
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

