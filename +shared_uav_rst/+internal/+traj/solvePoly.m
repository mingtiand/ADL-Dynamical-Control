function [p, J] = solvePoly(costWeight, T, constraints,stateSize,numSegments,segmentNumCoefficient,segmentOrder)
%This function is for internal use only. It may be removed in the future.
% Compute the optimal polynomial given time allocation and
% constrained segment derivatives
%     Returns
%        p is polynomial coefficients for each segments
%        J is the total cost
%
%     COSTWEIGHT is p vector indicating how much each order of 
%     derivative of the polynomial contribute to the cost. Only
%     considering jerk and snap cost.
%
%     Example for a 2 segment order-7 polynomial, costWeightIdx
%     [1 0 1 0] means that the cost includes 0-order derivative
%     and 2nd-order derivative equally. For minimum jerk costWeight is
%     [0 0 0 1 0 0 0 0].
%     T is a NumSegments vector contains time allocation for
%     each segment
%
%     CONSTRAINTS is a (numSegments+1)*segmentNumCoefficient/2
%     vector. For each segment, you can constrain
%     0:SegmentNumCoefficient/2-1 order of derivatives. For any
%     free constraints, specify as NaN
%
%     Example for a 2 segment order-3 polynomial, constraints
%     [1 0 1.5 NaN 2 0] limits the first waypoint to 1, its
%     derivative to 0, limits the last waypoint to 2, its
%     derivative to 0, the intermediate waypoint to 1.5, its
%     derivative is free.
%
%     STATESIZE is the total number of parameters to be solved. It is equal
%     to the numSegments*segmentNumCoefficient
%     
%     NUMSEGMENTS is the total number of segments. This is equal to the
%     number of waypoints-1
%     SEGMENTNUMCOEFFICIENT is the total number of coefficients per
%     segment. This is equal to the polynomial order + 1
%     
%     SEGMENTORDER is the polynomial order selected for the trajectory. For
%     minimum jerk, polynomial order is 7.

% Copyright 2021 The MathWorks, Inc.
%#codegen

numConstraints = sum(~isnan(constraints));

%Compute cost and boundary mapping matrices
Q = shared_uav_rst.internal.traj.constructQ(stateSize, numSegments,segmentNumCoefficient,segmentOrder,costWeight,T);
A = shared_uav_rst.internal.traj.constructA(T,stateSize, numSegments,segmentNumCoefficient,segmentOrder);

%Compute matrix to convert problem from constrained to unconstrained
%problem
M = shared_uav_rst.internal.traj.constructM(constraints,stateSize, numSegments,segmentNumCoefficient);

% Refer equations 15-17 in "Aggressive Flight of Fixed-Wing and Quadrotor
% Aircraft in Dense Indoor Environments" [1].
AInv = zeros(size(A));
for idx = 1:numSegments
    AHalfSize = segmentNumCoefficient/2;
    startIdx = (idx-1)*segmentNumCoefficient+1;
    blockRange = startIdx:startIdx+segmentNumCoefficient-1;
    ABlock = A(blockRange, blockRange);

    upperleft = inv(ABlock(1:AHalfSize, 1:AHalfSize));
    lowerright = inv(ABlock(1+AHalfSize:end, 1+AHalfSize:end));
    AInv(blockRange, blockRange) = [upperleft zeros(AHalfSize);
        -lowerright*ABlock(1+AHalfSize:end, 1:AHalfSize)*upperleft lowerright]; %#ok<MINV>
end
QPrime = AInv'*Q*AInv;
R = M*QPrime*M';

upper = numConstraints;
RPP = R(upper+1:end, upper+1:end);
RFP = R(1:upper, upper+1:end);
DF = constraints(~isnan(constraints));

DP = -RPP\(RFP'*DF);

D = [DF;DP];
d = M'*D;

%solve for the polynomial coefficients
p = A\d;

p = reshape(p, segmentNumCoefficient, numSegments);
p = p(end:-1:1, :);
p = p';

J = D'*R*D;
end
