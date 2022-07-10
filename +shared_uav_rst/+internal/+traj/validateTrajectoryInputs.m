function validateTrajectoryInputs(fcnName, waypoints, timePoints, numSamples)
%validateTrajInputs Validate the waypoints, timePoints and numSamples inputs

% Copyright 2021 The MathWorks, Inc.
%#codegen

% Check input validity
    validateattributes(waypoints, {'numeric'}, {'2d','nonempty','real','finite'}, fcnName,'waypoints');
    validateattributes(timePoints, {'numeric'}, {'nonempty','vector','real','finite','increasing','nonnegative'}, fcnName,'timePoints');
    validateattributes(numSamples, {'numeric'}, {'nonempty','scalar','real','finite','positive'}, fcnName,'numSamples');
end