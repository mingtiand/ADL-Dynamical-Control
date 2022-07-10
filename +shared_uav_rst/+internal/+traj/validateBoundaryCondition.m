function validateBoundaryCondition(trajType, fcnName, velBC, accBC, jerkBC, snapBC)
%This function is for internal use only. It may be removed in the future.
%validateBoundaryCondition Validate that boundary conditions name-value
%pairs

% Copyright 2021 The MathWorks, Inc.
%#codegen

% Check name-pair boundary conditions - real and finite
validateattributes(velBC, {'numeric'}, {'real'}, fcnName,'velocity boundary condition');
validateattributes(accBC, {'numeric'}, {'real'}, fcnName,'acceleration boundary condition');
validateattributes(jerkBC, {'numeric'}, {'real'}, fcnName,'jerk boundary condition');

coder.internal.errorIf(any(isinf(velBC),'all'), ['shared_uav_rst:' fcnName ':VelocityBCFinite']);
coder.internal.errorIf(any(isinf(accBC),'all'), ['shared_uav_rst:' fcnName ':AccelerationBCFinite']);
coder.internal.errorIf(any(isinf(jerkBC),'all'), ['shared_uav_rst:' fcnName ':JerkBCFinite']);
if strcmp(trajType,'snap')
    validateattributes(snapBC, {'numeric'}, {'real'}, fcnName,'snap boundary condition');
    coder.internal.errorIf(any(isinf(snapBC),'all'), ['shared_uav_rst:' fcnName ':SnapBCFinite']);    
end

end