function [p, t] = optimize(costWeightIdx, constraints, kt, minSegmentTime, ...
    maxSegmentTime, initGuess, stateSize, numSegments, segmentNumCoefficient, segmentOrder)
%This function is for internal use only. It may be removed in the future.
%OPTIMIZE Find the polynomial coefficients and the optimal time segment lengths
%   [P, T] = optimize(COSTWEIGHTIDX, CONSTRAINTS, KT,MINSEGMENTTIME, 
%   MAXSEGMENTTIME, INITGUESS, STATESIZE, NUMSEGMENTS,
%   SEGMENTNUMCOEFFICIENT, SEGMENTORDER) computes the polynomial segment 
%   coefficients, P, and the optimal time segment lengths, T, while 
%   minimizing the jerk/snap. When the time segment lengths are specified, 
%   the polynomial coefficients that minimize the jerk/snap are obtained 
%   using matrix manipulations. An iterative process is used to find the 
%   optimal time allocation or find the time segment lengths. The inputs 
%   to the function are the cost weight index, COSTWEIGHTIDX, to either 
%   minimize jerk or snap, the boundary conditions specified in, CONSTRAINTS, 
%   the time weight, KT, the lower bound on the time segment length, 
%   MINSEGMENTTIME, the upper bound on the time segment length,
%   MAXSEGMENTTIME, the initial guess for the time segment lengths,
%   INITGUESS, the total number of state variables, STATESIZE, the number
%   of polynomial segments, NUMSEGMENTS, the number of coefficients for 
%   each polynomial segment, SEGMENTNUMCOEFFICIENT and the polynomial order 
%   of the segments, SEGMENTORDER.

% Copyright 2021 The MathWorks, Inc.
%#codegen

    % Solver selected is damped BFGS gradient projection 
    solver = robotics.core.internal.DampedBFGSwGradientProjection;
    
    % Set solver parameters
    solver.ConstraintsOn = true;
    solver.RandomRestart = false;
    
    %Added this line to support code generation
    coder.varsize('initialGuess',[1,inf],[0,1]);
    initialGuess = initGuess;
    
    % Cost function 
    solver.CostFcn = @shared_uav_rst.internal.traj.trajectoryCost;  

    % Pass extra arguments as a struct to the solver
    args.constraints = constraints;
    args.kt = kt;
    args.costWeightIdx = costWeightIdx;
    args.stateSize = stateSize;
    args.numSegments = numSegments;
    args.segmentNumCoefficient = segmentNumCoefficient;
    args.segmentOrder = segmentOrder;
    args.cost = 0;
    args.grads = zeros(1,numel(initialGuess));            
    solver.ExtraArgs = args;
    
    % Random seed function
    % This is overridden due to codegen limitations and we are not using
    % random restart
    solver.RandomSeedFcn = @randfcn;
    
    % Specify the gradient function for the solver
    solver.GradientFcn = @gradient;
    
    % Specify the evaluation function for the solver
    solver.SolutionEvaluationFcn  = @solutionEval;
    
    % Set the constraint bounds for minimum and maximum segment time
    A1 = -eye(numel(initialGuess));
    A2 = eye(numel(initialGuess));        
   
    solver.ConstraintMatrix = [A1 A2];

    % If minimum segment time is specified as a scalar, replicate this to
    % all the time segments.
    if isscalar(minSegmentTime)
        b1 = -minSegmentTime*ones(numel(initialGuess),1);
    else
        % Use the minimum segment time specified as a vector
        b1 = -minSegmentTime;
    end

    % Same as variable b1
    if isscalar(maxSegmentTime)
        b2 = maxSegmentTime*ones(numel(initialGuess),1);
    else
        b2 = maxSegmentTime;
    end
    
    solver.ConstraintBound = [b1;b2];
    
    % Solve the optimization problem with the given initial guess.
    [t, ~] = solver.solve(initialGuess);
    
    % Initialize coefficients
    p = zeros(numSegments, segmentNumCoefficient, size(constraints, 2));
    
    % Compute optimal polynomial coefficients for each dimension from the
    % given time segment lengths
    for dimIdx = 1:size(constraints, 2)
        p(:, :, dimIdx) = shared_uav_rst.internal.traj.solvePoly(costWeightIdx, t, constraints(:,dimIdx), ...
            stateSize, numSegments, segmentNumCoefficient, segmentOrder);
    end
end

function err = gradient(T, args)
%gradient Gradient function for solver
    err = shared_uav_rst.internal.traj.computeJacobian(T, args.costWeightIdx, args.constraints, args);
end

function err = solutionEval(~, args)
%solutionEval Evaluation function for solver
    err = args.cost;
end

function z = randfcn(args)
%randfcn
% Since RandomRestart is set to false, this function is not required to be
% defined. But this function needs to be defined here to support code 
% generation.
    z = 0.1*ones(1,numel(args.numSegments));
end



