function [velBC, accBC, jerkBC, snapBC, timeOptim, timeWt, minSegmentTime, maxSegmentTime] = parseInputs(names, defaults, charInputs, trajType)
%parseInputs Parse inputs

% Copyright 2021 The MathWorks, Inc.
%#codegen

    parser = robotics.core.internal.NameValueParser(names, defaults);
    parse(parser, charInputs{:});
    velBC = parameterValue(parser, 'VelocityBoundaryCondition');
    accBC = parameterValue(parser, 'AccelerationBoundaryCondition');
    jerkBC = parameterValue(parser,'JerkBoundaryCondition');
    timeOptim = parameterValue(parser,'TimeAllocation');
    timeWt = parameterValue(parser,'TimeWeight');
    minSegmentTime = parameterValue(parser, 'MinSegmentTime');
    maxSegmentTime = parameterValue(parser, 'MaxSegmentTime');
    if strcmp(trajType,'snap')
        snapBC = parameterValue(parser, 'SnapBoundaryCondition');
    else
        snapBC = [];
    end

end