speedTimeConstant_s = 1;
initialSpeed_mPs    = 5;

headingTimeConstant_s = 1;

% Note that this has to be run after the path is specified (usually
% initialized in the controller)
v = calculatePathTangentVector(0,pathWidth_m,pathHeight_m);

initialHeading_deg    = (180/pi)*atan2(v(2),v(1));

initialXPosition_m = 0;
initialYPosition_m = 0;

clearvars v