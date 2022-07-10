function Atotal = constructA(T,stateSize, numSegments,segmentNumCoefficient,segmentOrder)
%This function is for internal use only. It may be removed in the future.
%constructA Matrix that maps between the polynomial coefficients and the endpoint derivatives

% Copyright 2021 The MathWorks, Inc.
%#codegen

%number of derivative constraints = number of polynomial coefficients by 2

%includes the 0th derivative which is the polynomial itself
numDerivatives = segmentNumCoefficient/2;

A0 = zeros(numDerivatives,segmentNumCoefficient);

AT = A0;
% Atotal matrix comes from equations 118-122 in "Aggressive Flight of Fixed-Wing and Quadrotor
% Aircraft in Dense Indoor Environments" [1].
Atotal = zeros(stateSize);

for segment = 1:numSegments
    for r = 0:numDerivatives-1
        
        prod = 1;
        for m = 0:r-1
            prod = prod*(r-m);
        end        
        A0(r+1,r+1) = prod;        
    end    
    
    for r = 0:numDerivatives - 1
        for col = 0:segmentOrder
            
            prod = 1;
            for m = 0:r-1
                prod = prod*(col-m);
            end
            
            AT(r+1,col+1) = prod*T(segment)^(col-r);
        end        
    end    
    Asegment = [A0;AT];
    rowIndex = (segment-1)*segmentNumCoefficient + 1:segment*segmentNumCoefficient;
    Atotal(rowIndex,rowIndex) = Asegment;
    
end