% Script to determine which variant is active and run the appropriate
% initilization script for that variant

switch VSSC_PLANT
    case 1
        nonlinearPlnt_cl_init
    case 2
        linearPlnt_cl_init
end