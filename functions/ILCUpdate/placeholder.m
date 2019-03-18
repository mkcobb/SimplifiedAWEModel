% Just a placeholder file to make sure github catches this directory


%Generate a vector containing the reference waypoint information. (Filled
%in with zeros for the other states to make the vector the right size)
stateReference = reshape([pathOnlyWaypoints, zeros(size(pathOnlyWaypoints,1),2)]',[4*size(pathOnlyWaypoints,1), 1]);

% The state vector of the previous iteration
xNominal = linPlntDisc.stateVector.data(:);

% The initial system states (will probably have to change this to the
% system states at the end of the previous iteration)
x0 = [initialXPosition_m;
    initialYPosition_m;
    initialSpeed_mPs;
    initialHeading_deg];
%The system input at the previous iteration
uNominal = squeeze(linPlntDisc.ctrlInput.Data);

% Difference in initial cond. (next sim starts at xFinal of current sim)
deltax0 = tsc.stateVector.data(:,:,end)-tsc.stateVector.data(:,:,1);
stateReference = stateReference - xNominal;


optimizationReference = stateReference - F*deltax0;

