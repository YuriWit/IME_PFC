classdef (Abstract) AbstractBodyTarget < handle
    
    methods (Abstract)
        pointTargets = getPointTargets(obj)
        update(obj, dt)
    end

end

