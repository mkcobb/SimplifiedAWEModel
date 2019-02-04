function [linPlnt,linPlntDisc] = linearizePlant(decimation,pathStep,tsc)
% Function to calculate the path-linearized version of a plant
% function returns the struct linPlnt which contains elements A, B, C, D,
% stateVector, ctrlInput.  These are all timeseries where the "time" is
% actually the path variable.

simulationTimeStep_s = pathStep; % This is really the path parameter step

numStates   = numel(tsc.stateVector.data(:,:,1));          % get the number of states in the model
numInputs   = numel(tsc.headingSetpoint_rad.data(:,:,1));  % Get the number of inputs

% Get the indices of the points to linearize around
stepsInx    = 1:decimation:numel(tsc.currentPathPosition_none.data);
% Include the last point available
if stepsInx(end)~=numel(tsc.currentPathPosition_none.data)
    stepsInx = [stepsInx numel(tsc.currentPathPosition_none.data)];
end
numSteps    = numel(stepsInx);

% Set up the lookup tables that will comprise the linear path-parameterized plant
linPlnt.A = timeseries(...
    nan([numStates numStates numSteps]),...
    tsc.currentPathPosition_none.data(stepsInx));
linPlnt.B = timeseries(...
    nan([numStates numInputs numSteps]),...
    tsc.currentPathPosition_none.data(stepsInx));

linPlnt.C = linPlnt.A;
linPlnt.D = linPlnt.B;

% Get the points in state space to linearize around
linPlnt.stateVector = timeseries(...
    tsc.stateVector.data(:,:,stepsInx),...
    tsc.currentPathPosition_none.data(stepsInx));
% Get the control inputs to linearize around
linPlnt.ctrlInput   = timeseries(...
    tsc.headingSetpoint_rad.data(:,:,stepsInx),...
    tsc.currentPathPosition_none.data(stepsInx));
% Get the staring point for the path projection
projectionStartPoints = tsc.currentPathPosition_none.data(stepsInx);

linPlntDisc = linPlnt;

for ii = 1:numSteps
    % Set the initial condition on the path projection
    initialPathPosition_none =  projectionStartPoints(ii);
    % Run linmod

    x = linmod('linearizationPlant_cm',...
        linPlnt.stateVector.data(:,:,ii),... 
        linPlnt.ctrlInput.data(ii));
    % Only keep the ones that don't include the discrete state (unit delay)
    linPlnt.A.data(:,:,ii) = x.a(1:numStates,1:numStates);
    linPlnt.B.data(:,:,ii) = x.b(1:numStates);
    linPlnt.C.data(:,:,ii) = x.c(1:numStates,1:numStates);
    linPlnt.D.data(:,:,ii) = x.d(1:numStates);

    discPlnt = ss(linPlnt.A.data(:,:,ii),linPlnt.B.data(:,:,ii),...
        linPlnt.C.data(:,:,ii),linPlnt.D.data(:,:,ii));
    discPlnt = c2d(discPlnt,pathStep);
    
    linPlntDisc.A.data(:,:,ii) = discPlnt.A;
    linPlntDisc.B.data(:,:,ii) = discPlnt.B;
    linPlntDisc.C.data(:,:,ii) = discPlnt.C;
    linPlntDisc.D.data(:,:,ii) = discPlnt.D;
end

end