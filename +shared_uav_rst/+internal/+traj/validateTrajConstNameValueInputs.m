function validateTrajConstNameValueInputs(fcnName, minSegmentTime, maxSegmentTime, timeWt, timeOptim)
%validateTrajConstNameValueInputs Validate the constant name-value arguments
% Validate the minimum segment time, maximum segment time, time weight
% and time optimization property

% Copyright 2021 The MathWorks, Inc.
%#codegen

% Check properties validity
    validateattributes(minSegmentTime, {'numeric'}, {'nonempty','vector','real','finite','positive'}, fcnName,'minimum segment time');
    validateattributes(maxSegmentTime, {'numeric'}, {'nonempty','vector','real','finite','positive','>',minSegmentTime}, fcnName,'maximum segment time');
    validateattributes(timeWt, {'numeric'}, {'nonempty','scalar','real','finite','nonnegative'}, fcnName,'time weight');
    validateattributes(timeOptim, {'logical'}, {'nonempty'}, fcnName,'time optimization');  
