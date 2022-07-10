function validateInputSize(trajType, fcnName, timePoints, numWaypoints, numDimensions, velBC, accBC, jerkBC, snapBC)
%This function is for internal use only. It may be removed in the future.
%validateInputSize Validate that the inputs are of the correct size

% Copyright 2021 The MathWorks, Inc.
%#codegen

% Input size checks
coder.internal.errorIf(numWaypoints < 2, ['shared_uav_rst:' fcnName ':WaypointsTooFew']);
coder.internal.errorIf(length(timePoints) ~= numWaypoints, ['shared_uav_rst:' fcnName ':WayPointMismatch']);
coder.internal.errorIf(size(velBC,1) ~= numDimensions || size(velBC,2) ~= numWaypoints, ['shared_uav_rst:' fcnName ':WaypointVelocityBCDimensionMismatch']);
coder.internal.errorIf(size(accBC,1) ~= numDimensions || size(accBC,2) ~= numWaypoints, ['shared_uav_rst:' fcnName ':WaypointAccelerationBCDimensionMismatch']);
coder.internal.errorIf(size(jerkBC,1) ~= numDimensions || size(jerkBC,2) ~= numWaypoints, ['shared_uav_rst:' fcnName ':WaypointJerkBCDimensionMismatch']);

if strcmp(trajType,'snap')
    coder.internal.errorIf(size(snapBC,1) ~= numDimensions || size(snapBC,2) ~= numWaypoints, ['shared_uav_rst:' fcnName ':WaypointSnapBCDimensionMismatch']);
end