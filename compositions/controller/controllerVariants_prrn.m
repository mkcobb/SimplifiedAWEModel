% Script that figures out which variant is active and runs the appropriate
% init script for that variant.  This should be run before simulation.

switch VSSC_CONTROLLER
    case 1
        purePursuitCtrl_cl_init
    case 2
        openLoopPathDomainCtrl_cl_init
    case 3
        purePursuitWithWarpingCtrl_cl_init
    case 4
        openLoopPathDomainCtrl_cl_init
end