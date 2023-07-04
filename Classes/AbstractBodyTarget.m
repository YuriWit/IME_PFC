classdef (Abstract) AbstractBodyTarget < handle
    
    methods (Abstract)
        pointTargets = getPointTargets(obj)
        forceUpdate(obj,newPosition, newVelocity, dt)
        update(obj, dt)
    end

end

