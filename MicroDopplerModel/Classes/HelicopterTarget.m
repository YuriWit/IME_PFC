classdef HelicopterTarget < RadarTarget
    
    properties
        Position
        Velocity
        
        Body
        Blades
    end
    
    methods
        function obj = HelicopterTarget(p)
            bp.c = p.c;
            bp.fc = p.fc;
            bp.meanRCS = p.meanRCS;
            Body = PointTraget()
            params.meanRCS = [params.bodyRadarCrossSection ...
                params.bladeRadarCrossSection*ones(1,params.numberOfBlades)];
            obj@RadarTarget(params);
            obj.NumberOfBlades = params.numberOfBlades;
            obj.BladeLength = params.bladeLength;
            obj.BladeAngularVelocity = params.bladeAngularVelocity;
        end

        function pointTargets = getPointTargets(obj, t)
            pointTargets = zeros(100,3,2);
        end

        function [scattersPosition, scattersVelocity] = targetMotion(obj, dt)
            obj.Time = obj.Time + dt;
            [bodyPosition, bodyVelocity] = obj.Platform(dt);
            
            % Initialize cell arrays for positions and velocities
            scattersPosition = cell(1, obj.NumberOfBlades+1);
            scattersVelocity = cell(1, obj.NumberOfBlades+1);
        
            % The first scatter is the helicopter body
            scattersPosition{1} = bodyPosition;
            scattersVelocity{1} = bodyVelocity;
            
            % Calculate the rotation matrix from the angular velocity vector
            rotationAxis = obj.BladeAngularVelocity / norm(obj.BladeAngularVelocity);  % normalize to get rotation axis
            rotationMatrix = axang2rotm([rotationAxis', 0]);  % 0 angle since we just want to align frames
            
            % Calculate the position and velocity of each blade tip
            for i = 1:obj.NumberOfBlades
                % The blade rotates in a circle around the body
                angle = norm(obj.BladeAngularVelocity) * obj.Time + (i-1) * (2*pi/obj.NumberOfBlades);  % Add phase offset for each blade
                positionOffset = obj.BladeLength * [cos(angle); sin(angle); 0];  % Blades rotate in x-y plane in body frame
        
                % Transform position offset to global frame and add to body position
                positionOffsetGlobal = rotationMatrix * positionOffset;
                bladePosition = bodyPosition + positionOffsetGlobal;
        
                % The velocity is the derivative of the position
                velocityOffset = norm(obj.BladeAngularVelocity) * obj.BladeLength * [-sin(angle); cos(angle); 0];  % The derivative of cos is -sin and vice versa
                velocityOffsetGlobal = rotationMatrix * velocityOffset;
                bladeVelocity = bodyVelocity + velocityOffsetGlobal;
        
                % Store in cell arrays
                scattersPosition{i+1} = bladePosition;
                scattersVelocity{i+1} = bladeVelocity;
            end
        end


        
        function returnVal = getScattersMotion(obj)
            numScatters = length(obj.Scatters.MeanRCS);
            returnVal = zeros(numScatters,3,2);
            for i=1:numScatters
                returnVal(i,:,:) = [obj.Position obj.Velocity];
            end
        end
    end
end

