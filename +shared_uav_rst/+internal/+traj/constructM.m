function M = constructM(constraints,stateSize, numSegments,segmentNumCoefficient)
%This function is for internal use only. It may be removed in the future.
%constructM Duplicate intermediate waypoint derivative and continuity matrix
% Matrix M duplicates each intermediate waypoint derivative value to appear 
% both at the end of one segment and at the beginning of the subsequent 
% segment. This maintains continuity at the intermediate waypoints

% Copyright 2021 The MathWorks, Inc.
%#codegen

% M matrix same as the C matrix in equation 11 in "Polynomial
% Planning for Aggressive Quadrotor Flight in Dense Indoor Environments" [2].
numDerivatives = segmentNumCoefficient/2;
xCons = constraints;

M1 = zeros(numel(xCons),numel(xCons));

fixedBCIdx = find(~isnan(xCons));
freeBCIdx = find(isnan(xCons));

for k = 1:numel(fixedBCIdx)    
    M1(fixedBCIdx(k),k) = 1;    
end

freeIdx = 1;

for kk = numel(fixedBCIdx)+1:size(M1,1)        
    M1(freeBCIdx(freeIdx),kk) = 1;    
    freeIdx = freeIdx + 1;
end

%each segment has numDerivatives as constraints at each endpoint. 
Mcontinuity = zeros(stateSize,(numSegments+1)*numDerivatives);
xConsAppended = zeros(1,stateSize);
for row = 1:stateSize
    %the boundary conditions of a segments end point is equal to the
    %starting boundary conditions of the next segment. Continuity
    %enforcement
    segmentNumber = ceil(row/(2*numDerivatives));
    col = row - (segmentNumber - 1)*numDerivatives;
    Mcontinuity(row,col) = 1;
    xConsAppended(row) = xCons(col); 
end
%size(Mcontinuity)
%size(M1)
M = (Mcontinuity*M1)';

