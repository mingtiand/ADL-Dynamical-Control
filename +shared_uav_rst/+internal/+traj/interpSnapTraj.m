function [q,qd,qdd,qddd,qdddd,toa,tSamples] = interpSnapTraj(pp,timePoints,numSamples)
%This function is for internal use only. It may be removed in the future.
%interpSnapTraj Interpolate to polynomial coefficients with the specified
% number of samples

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
    qdddd = q;
    
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
            % Compute first derivative of the polynomial. Initialization
            % added to support codegen.
            polydCoef = zeros(1,9);
            polydCoef = polydCoef + polyder(pp(segmentNum(kk),:,k));
            polyd = mkpp([0 timePoints(segmentNum(kk)+1)-timePoints(segmentNum(kk))],polydCoef);
            % Interpolate velocities
            qd(k,kk) = ppval(polyd,delT);
            polyddCoef = zeros(1,8);
            polyddCoef = polyddCoef + polyder(polydCoef);
            % Compute second derivative
            polydd = mkpp([0 timePoints(segmentNum(kk)+1)-timePoints(segmentNum(kk))],polyddCoef);
            % Interpolate acceleration
            qdd(k,kk) = ppval(polydd,delT);
            polydddCoef = zeros(1,7);
            polydddCoef = polydddCoef + polyder(polyddCoef);
            % Compute third derivative/jerk
            polyddd = mkpp([0 timePoints(segmentNum(kk)+1)-timePoints(segmentNum(kk))],polydddCoef);
            % Interpolate jerk
            qddd(k,kk) = ppval(polyddd,delT);
            polyddddCoef = zeros(1,6);
            polyddddCoef = polyddddCoef + polyder(polydddCoef);
            % Compute fourth derivative/snap
            polydddd = mkpp([0 timePoints(segmentNum(kk)+1)-timePoints(segmentNum(kk))],polyddddCoef);
            % Interpolate snap
            qdddd(k,kk) = ppval(polydddd,delT);
            
        end        
    end
end