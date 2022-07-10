function Jac = computeJacobian(T,costWeightIdx,constraint,args)
%This function is for internal use only. It may be removed in the future.    
%computeJacobian Computes the Jacobian matrix
% This function computes the Jacobian of the cost function using a
% numerical method.

% Copyright 2021 The MathWorks, Inc.
%#codegen

Jac = zeros(1,numel(T));
%this value was changed from 1e-9 to 1e-5 to match fmincon results
delta = 1e-5;
scalar = 1/(2*delta);

    for kk = 1:numel(T)
        deltavec = zeros(numel(T),1);
        deltavec(kk) = delta;
        deltaTP = T + deltavec;
        deltaTN = T - deltavec;
        er1 = 0;er2 = 0;
        %compute the perturbed costs
        for idx = 1:size(args.constraints,2)
            [~,er3] = shared_uav_rst.internal.traj.solvePoly(costWeightIdx,deltaTP,constraint(:,idx),args.stateSize,args.numSegments,args.segmentNumCoefficient,args.segmentOrder);
            [~,er4] = shared_uav_rst.internal.traj.solvePoly(costWeightIdx,deltaTN,constraint(:,idx),args.stateSize,args.numSegments,args.segmentNumCoefficient,args.segmentOrder);
            er1 = er1 + er3;
            er2 = er2 + er4;
        end
    
        er1 = er1 + args.kt*sum(deltaTP);
        er2 = er2 + args.kt*sum(deltaTN);
    
        Jac(kk) = (er1-er2)*scalar;    
    end
end