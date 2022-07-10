function Qtotal = constructQ(stateSize, numSegments,segmentNumCoefficient,segmentOrder,costWeight, T)
%This function is for internal use only. It may be removed in the future.
%constructQ Quadratic cost matrix

% Copyright 2021 The MathWorks, Inc.
%#codegen

prod = 1;
r = 0;
Q = zeros(segmentNumCoefficient);
Qsum = Q;
% Qtotal matrix comes from equation 114 in "Aggressive Flight of Fixed-Wing and Quadrotor
% Aircraft in Dense Indoor Environments" [1].
Qtotal = zeros(stateSize);
for segment = 1:numSegments
    segmentTime = T(segment); 
    for k = 0:segmentOrder
        for row = r:segmentOrder
            for col = r:segmentOrder
                for m = 0:r-1
                    prod = prod*(row-m)*(col-m);
                end
                powerTerm = row + col - 2*r + 1;
                Q(row+1,col+1) = prod*(segmentTime^powerTerm)/powerTerm;
                prod = 1;
            end
        end
        
        Qsum = Qsum + costWeight(r+1)*Q;
        r = r + 1;
        Q = zeros(segmentNumCoefficient);
        
    end
    
    rowIndex = (segment-1)*segmentNumCoefficient + 1:segment*segmentNumCoefficient;
    Qtotal(rowIndex,rowIndex) = Qsum;
    Qsum = zeros(segmentNumCoefficient);
    r = 0;
end

