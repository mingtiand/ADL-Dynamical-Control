function [q,qd,qdd,qddd,toa,tSamples] = interpJerkTraj(pp,timePoints,numSamples)
%This function is for internal use only. It may be removed in the future.
%interpJerkTraj Interpolate to polynomial coefficients with the specified
% number of samples

%%%%%%%%%%%%%%%%%%%%%%  UPDATE THIS TO MAKE IT CODEGEN   %%%%%%%%%%%%%%%%%%

% Copyright 2021 The MathWorks, Inc.
%#codegen

    % Linearly spaced time samples
    tSamples = linspace(timePoints(1),timePoints(end),numSamples);
    % Time of arrival at the waypoints
    toa = timePoints;
    
    % Trajectory dimension
    numDim = size(pp,3);
    
    % Initialize trajectory  
    q = zeros(numDim,numSamples);
    qd = q;
    qdd = q;
    qddd = q;
    
    % Find the segment number in which the time samples fall
    [~,~,segmentNum] = histcounts(tSamples,timePoints);
    
    for k = 1:numDim
        for kk = 1:numSamples
            
            % Compute distance of time sample from the segment start time
            delT = tSamples(kk) - timePoints(segmentNum(kk));
            % Construct piece-wise polynomial from pp
            poly = mkpp([0 timePoints(segmentNum(kk)+1)-timePoints(segmentNum(kk))],pp(segmentNum(kk),:,k));
            % Interpolate position
            q(k,kk) = ppval(poly,delT);
            % Compute first derivative of the polynomial. The initialization
            % line is added for codegen 
            polydCoef = zeros(1,5);%mingtiand zeros(1,7)
            %size(polydCoef)
            polydCoef = polydCoef + polyder(pp(segmentNum(kk),:,k));
            polyd = mkpp([0 timePoints(segmentNum(kk)+1)-timePoints(segmentNum(kk))],polydCoef);
            % Interpolate velocities
            qd(k,kk) = ppval(polyd,delT);
            polyddCoef = zeros(1,4);%mingtiand zeros(1,6)
            polyddCoef = polyddCoef + polyder(polydCoef);           
            % Compute second derivative
            polydd = mkpp([0 timePoints(segmentNum(kk)+1)-timePoints(segmentNum(kk))],polyddCoef);
            % Interpolate acceleration
            qdd(k,kk) = ppval(polydd,delT);
            polydddCoef = zeros(1,3);%mingtiand zeros(1,6)
            polydddCoef = polydddCoef + polyder(polyddCoef);
            % Compute third derivative/jerk
            polyddd = mkpp([0 timePoints(segmentNum(kk)+1)-timePoints(segmentNum(kk))],polydddCoef);
            % Interpolate jerk
            qddd(k,kk) = ppval(polyddd,delT);
            
        end        
    end
end