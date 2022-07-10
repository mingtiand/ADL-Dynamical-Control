function orderedConstraints = orderConstraints(waypoints, velBC, accBC, jerkBC, snapBC, numDerivatives)
%orderedConstraints Re-order the constraints in desired format. Each column 
% corresponds to constraints for a particular dimension.

% Copyright 2021 The MathWorks, Inc.
%#codegen

    % Note the change in order. The waypoints are transposed before passing
    % to this function.
    numWaypoints = size(waypoints,1);
    trajDimension = size(waypoints,2);
    
    % Each column will correspond to all the constraints for each
    % dimension. For example, for a 2D problem with 3 waypoints, and for
    % jerk
    % constraints = [x1 dx1 ddx1 dddx1 x2 dx2 ddx2 dddx2 x3 dx3 ddx3 dddx3; 
    %                y1 dy1 ddy1 dddy1 y2 dy2 ddy2 dddy2 y3 dy3 ddy3 dddy3]'
    % and for snap
    % constraints = [x1 dx1 ddx1 dddx1 ddddx1 x2 dx2 ddx2 dddx2 ddddx2 x3 dx3 ddx3 dddx3 ddddx3; 
    %                y1 dy1 ddy1 dddy1 ddddy1 y2 dy2 ddy2 dddy2 ddddy2 y3 dy3 ddy3 dddy3 ddddy3]'
    constraints = zeros(numDerivatives*numWaypoints,trajDimension);
    
  
    for k = 1:numWaypoints
        if isequal(numDerivatives,3) 
            constraints(numDerivatives*(k-1)+1:numDerivatives*k,:) = [waypoints(k,:);velBC(k,:);accBC(k,:)];
        elseif isequal(numDerivatives,4) 
            constraints(numDerivatives*(k-1)+1:numDerivatives*k,:) = [waypoints(k,:);velBC(k,:);accBC(k,:);jerkBC(k,:)];
        else
            constraints(numDerivatives*(k-1)+1:numDerivatives*k,:) = [waypoints(k,:);velBC(k,:);accBC(k,:);jerkBC(k,:);snapBC(k,:)];
        end
    end
    orderedConstraints = constraints;
end