function [q, qd, qdd, qddd, pp, tpts, tSamples] = minjerkpolytraj(waypoints, timePoints, numSamples, varargin)
%MINJERKPOLYTRAJ Generate minimum jerk trajectories through multiple waypoints
%   [Q, QD, QDD, QDDD, PP, TPTS, TSAMPLES] = minjerkpolytraj(WAYPOINTS, TIMEPOINTS, NUMSAMPLES) generates
%   a minimum jerk polynomial trajectory that achieves a given set of input
%   waypoints with their corresponding time points. The function outputs
%   positions, Q, velocities, QD, accelerations, QDD, and jerks, QDDD at
%   the given number of samples, NUMSAMPLES. The function also returns the
%   piecewise polynomial form, PP, of the polynomial trajectory with respect 
%   to time, the time points, TPTS, and the sample times, TSAMPLES.
%
%      WAYPOINTS  - Waypoints of trajectory, specified as an N-by-P matrix.
%                   N is the dimension of the trajectory and P is the
%                   number of waypoints.
%
%      TIMEPOINTS - Time points for waypoints of trajectory, specified as a
%                   P-element row vector.
%
%      NUMSAMPLES - Number of samples, M, specified as a scalar.
%
%   [Q, QD, QDD, QDDD, PP, TPTS, TSAMPLES] = minjerkpolytraj(___, Name, Value)
%   specifies additional parameters as Name-Value pair arguments using any
%   combination of the previous syntaxes. Use NaN to specify a free
%   boundary condition.
%
%      'VelocityBoundaryCondition'    -  The velocity boundary conditions,
%                                        specified as an N-by-P matrix of
%                                        velocities at each waypoint. By
%                                        default, this is assumed to be
%                                        zero at the boundary waypoints and
%                                        NaN at the intermediate waypoints.
%
%      'AccelerationBoundaryCondition'-  The acceleration boundary
%                                        conditions, specified as an N-by-P
%                                        matrix of accelerations at each
%                                        waypoint. By default, this is
%                                        assumed to be zero at the boundary
%                                        waypoints and NaN at the intermediate
%                                        waypoints.
%
%     'JerkBoundaryCondition'         -  The jerk boundary conditions,
%                                        specified as an N-by-P matrix of
%                                        jerks at each waypoint. By
%                                        default, this is assumed to be
%                                        zero at the boundary waypoints and
%                                        NaN at the intermediate waypoints.
%
%     'TimeAllocation'              -    The time allocation flag
%                                        specified as a logical value. By
%                                        default this is assumed to be
%                                        false. Enable this flag to
%                                        optimize a combination of jerk and
%                                        total segment time cost.
%
%     'TimeWeight'                    -  The time weight for time
%                                        allocation. By default this is
%                                        assumed to be 100.
%
%     'MinSegmentTime'                -  The minimum time segment length
%                                        specified as a positive scalar or
%                                        a (P-1)-element row vector. By
%                                        default this is assumed to be 0.1.
%
%     'MaxSegmentTime'                -  The maximum time segment length
%                                        specified as a positive scalar or
%                                        a (P-1)-element row vector. By
%                                        default this is assumed to be 5.
%
%   [Q, QD, QDD, QDDD, PP, TPTS, TSAMPLES] = minjerkpolytraj(WAYPOINTS, TIMEPOINTS, NUMSAMPLES, 'TimeAllocation',true)
%   minimizes jerk and optimizes the time segment lengths between the
%   waypoints. In this case, TIMEPOINTS, is treated as an initial guess for
%   the time of arrival at the waypoints.
%
%   Example:
%      % Define position waypoints, time points and number of samples
%
%      wpts = [2 5 8 4];
%      tpts = [1 2 3 5];
%      numsamples = 50;
%      % Compute trajectory using default boundary conditions
%      [q, qd, qdd, qddd, pp, timepoints, tsamples] = minjerkpolytraj(wpts, tpts, numsamples);
%
%      % Plot results. The x's are waypoints, and the lines are the derived
%      % polynomial curves
%      plot(tsamples, q)
%      hold all
%      plot(timepoints, wpts, 'x')
%      hold off
%
%      See also BSPLINEPOLYTRAJ, QUINTICPOLYTRAJ, CUBICPOLYTRAJ, TRAPVELTRAJ


%   References:
%
%   [1] Richter, Charles Andrew, Bry, Adam P. and Roy, Nicholas "Aggressive
%       Flight of Fixed-Wing and Quadrotor Aircraft in Dense Indoor Environments".
%       International Journal of Robotics Research, 2015.
%   [2] Richter, Charles Andrew, Bry, Adam P. and Roy, Nicholas "Polynomial
%       Planning for Aggressive Quadrotor Flight in Dense Indoor Environments".
%       In Proceedings of the International Symposium of Robotics Research
%       (ISRR 2013).

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    narginchk(3,17)
    numInputArgs = nargin;

    % Convert strings to chars case by case for codegen support
    charInputs = shared_uav_rst.internal.traj.stringToChar(numInputArgs,varargin{:});

    % Check input validity
    FCN_NAME = 'minjerkpolytraj';
    shared_uav_rst.internal.traj.validateTrajectoryInputs(FCN_NAME, waypoints, timePoints, numSamples);

    % Ensure timePoints is a row vector
    timePoints = timePoints(:)';

    % Dimension of the waypoint
    numDimensions = size(waypoints,1);
    % Number of waypoints
    numWaypoints = size(waypoints,2);

    % Default boundary conditions. Zero at start and end waypoints. NaN at
    % intermediate waypoints
    velBCDefault = [zeros(numDimensions,1) nan(numDimensions,numWaypoints-2) zeros(numDimensions,1)];
    accBCDefault = [zeros(numDimensions,1) nan(numDimensions,numWaypoints-2) zeros(numDimensions,1)];
    jerkBCDefault = [zeros(numDimensions,1) nan(numDimensions,numWaypoints-2) zeros(numDimensions,1)];

    % Constant name-value inputs
    TIME_DEFAULT_WEIGHT = 100;
    MIN_SEGMENT_TIME = 0.1;
    MAX_SEGMENT_TIME = 5;
    TIME_ALLOCATION = false;

    % Parse inputs
    names = {'VelocityBoundaryCondition', 'AccelerationBoundaryCondition', ...
             'JerkBoundaryCondition', 'TimeAllocation', 'TimeWeight', 'MinSegmentTime', 'MaxSegmentTime'};
    defaults = {velBCDefault, accBCDefault, jerkBCDefault, TIME_ALLOCATION, ...
                TIME_DEFAULT_WEIGHT, MIN_SEGMENT_TIME, MAX_SEGMENT_TIME};
    trajType = 'jerk';
    [velBC, accBC, jerkBC, ~, timeOptim, timeWt, minSegmentTime, maxSegmentTime] = shared_uav_rst.internal.traj.parseInputs(names, defaults, charInputs, trajType);
    
    % Validate constant name-value input arguments
    shared_uav_rst.internal.traj.validateTrajConstNameValueInputs(FCN_NAME,minSegmentTime, maxSegmentTime, timeWt, timeOptim);
    
    % Input size checks
    shared_uav_rst.internal.traj.validateInputSize(trajType, FCN_NAME, timePoints, numWaypoints, numDimensions, velBC, accBC, jerkBC, []);
    % Validate name-value pair boundary conditions
    
    shared_uav_rst.internal.traj.validateBoundaryCondition(trajType, FCN_NAME, velBC, accBC, jerkBC, []);

    % Set cost weight to minimize jerk. The n-th element of the cost weight
    % vector corresponds to the (n-1)-th derivative of the polynomial. Index
    % 4 corresponds to jerk.
    COST_WEIGHT = [0 0 0 1 0 0];%mingtiand[0 0 0 1 0 0 0 0]
    
    % Order of the polynomial selected for minimum jerk trajectory
    SEGMENT_ORDER = 5;%mingtiand7

    % Number of derivatives of the polynomial trajectory considered
    numDerivatives = (SEGMENT_ORDER + 1)/2;
    
    % Re-order constraints in desired format. Note that the input is
    % transposed before passing as arguments. Pass empty matrix for snap
    % boundary conditions.
    constraints = shared_uav_rst.internal.traj.orderConstraints(waypoints', velBC', accBC', jerkBC', [], numDerivatives);
    
    % Compute the polynomial segment coefficients and time of arrival
    [ppMatrix, timeOfArrival] = shared_uav_rst.internal.traj.computePolyCoefAndTimeOfArrival(waypoints, timePoints, COST_WEIGHT, ...
                                                                                       constraints, timeWt, minSegmentTime, ...
                                                                                       maxSegmentTime, SEGMENT_ORDER, timeOptim, FCN_NAME);
    
    % Convert to piecewise polynomial form
    pp = shared_uav_rst.internal.traj.convertToPPForm(ppMatrix,numWaypoints,timeOfArrival);

    % Compute outputs at time samples from the polynomial coefficients by
    % interpolation
    
    [q, qd, qdd, qddd, tpts, tSamples] = shared_uav_rst.internal.traj.interpJerkTraj(ppMatrix, timeOfArrival, numSamples);
    
end
