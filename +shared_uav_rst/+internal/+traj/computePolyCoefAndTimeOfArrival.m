function [pp, timeOfArrival] = computePolyCoefAndTimeOfArrival(waypoints, timePoints, COST_WEIGHT, ...
                                                               constraints, timeWt, minSegmentTime, ...
                                                               maxSegmentTime, SEGMENT_ORDER, timeOptim, fcnName)
%computePolyCoefAndTimeOfArrival Compute polynomial segment coefficients and time of arrival
% This function computes the polynomial coefficients and the time of
% arrival given the cost weight, time weight, minimum segment weight,
% maximum segment weight and the segment order. If time optimization
% property is set to true, then this function returns the optimal polynomial
% coefficients and the time of arrival.

% Copyright 2021 The MathWorks, Inc.
%#codegen

% Number of dimensions
    numDimensions = size(waypoints,1);

    % Number of trajectory segments
    numSegments = size(waypoints,2)-1;

    % Number of coefficients for each segment. Each segment has tow end
    % waypoints with boundary conditions for position, velocity,
    % acceleration and jerk
    segmentNumCoefficient = SEGMENT_ORDER+1;

    % Total number of states or boundary conditions
    stateSize = segmentNumCoefficient*numSegments;

    % Initial guess for the time segment lengths when time optimization is
    % selected
    initialGuess = diff(timePoints);

    if timeOptim

        %check segment bounds
        coder.internal.errorIf(min(initialGuess) < minSegmentTime, ['shared_uav_rst:' fcnName ':MinSegmentTimeExceed']);
        coder.internal.errorIf(max(initialGuess) > maxSegmentTime, ['shared_uav_rst:' fcnName ':MaxSegmentTimeExceed']);


        % Perform time optimization and jerk minimization
        [pp, tSegments] = shared_uav_rst.internal.traj.optimize(COST_WEIGHT, constraints, ...
                                                                timeWt, minSegmentTime, maxSegmentTime, initialGuess, stateSize,...
                                                                numSegments, segmentNumCoefficient, SEGMENT_ORDER);
        timeOfArrival = [0 cumsum(tSegments)'];
    else

        % Solve the polynomial coefficients. Since the time of arrival is
        % specified, no time optimization or allocation is required here.
        % The polynomial coefficients are obtained by simple matrix
        % manipulations and inversion
        pp = zeros(numSegments, segmentNumCoefficient, size(constraints, 2));
        for dimIdx = 1:numDimensions
            pp(:, :, dimIdx) = shared_uav_rst.internal.traj.solvePoly(COST_WEIGHT, diff(timePoints), ...
                                                                      constraints(:,dimIdx), stateSize, numSegments, segmentNumCoefficient, SEGMENT_ORDER);
        end

        % timeofArrival is same as the specified time points
        timeOfArrival = timePoints;
    end
end
