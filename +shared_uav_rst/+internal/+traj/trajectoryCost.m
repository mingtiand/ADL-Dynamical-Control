function [J,a,b,args] = trajectoryCost(T,args)
%This function is for internal use only. It may be removed in the future.
%trajectoryCost Compute the cost function.
% This consists of two parts. First one computes jerk/snap cost and the
% second computes the time cost

% Copyright 2021 The MathWorks, Inc.
%#codegen

J = 0;

for idx = 1:size(args.constraints,2)

    %Cost per dimension
    [~, Jidx] = shared_uav_rst.internal.traj.solvePoly(args.costWeightIdx, T, ...
        args.constraints(:,idx),args.stateSize,args.numSegments, ...
        args.segmentNumCoefficient,args.segmentOrder);

    % Total cost for all the trajectory dimensions
    J = J + Jidx;
end
% Add the time cost
J = J + args.kt*sum(T);
args.cost = J;

% a and b refer to the weight matrix and the Jacobian. We are not
% computing Jacobian here and the weight matrix is not relevant to our
% cost function. But the solver expects this function to have the given
% signature.

a = [];

b = [];
end