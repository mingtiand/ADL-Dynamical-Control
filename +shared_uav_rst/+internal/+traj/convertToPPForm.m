function ppForm = convertToPPForm(pp,numWpts,timePoints)
%This function is for internal use only. It may be removed in the future.
%convertToPPForm Convert to piece-wise polynomial form.

% Copyright 2021 The MathWorks, Inc.
%#codegen

numSegments = numWpts - 1;
trajDim = size(pp,3);

%initialize polynomial matrix. Number of rows will be number of segments 
% multiplied by trajectory dimension.
rows = trajDim*numSegments;
cols = size(pp,2);

ppMatrix = zeros(rows,cols);

% reshape the pp matrix to required form 
for i = 1:numSegments    
    for k = 1:trajDim        
        ppMatrix((i-1)*trajDim + k,:) = pp(i,:,k);
    end
end

%Convert to piece-wise polynomial form
ppForm = mkpp(timePoints,ppMatrix,trajDim);
