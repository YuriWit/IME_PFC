classdef Test < handle
    %TEST Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = Test(inputArg)
            %TEST Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg;
        end
        
        function update(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.Property1 = obj.Property1 + inputArg;
        end
    end
end

