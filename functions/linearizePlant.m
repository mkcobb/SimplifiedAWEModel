function linPlnt = linearizePlant(decimation,tsc)
% Function to calculate the path-linearized version of a plant
% function returns the struct linPlnt which contains elements A, B, C, D,
% stateVector, ctrlInput
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

simulationTimeStep_s = 0.001; % This is really the path parameter step

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

for ii = 1:numSteps
    % Set the initial condition on the path projection
    initialPathPosition_none =  projectionStartPoints(ii);
    % Run linmod
    try
    x = linmod('linearizationPlant_cm',...
        linPlnt.stateVector.data(:,:,ii),... % Append for the discrete state (unit delay)
        linPlnt.ctrlInput.data(ii));
    % Only keep the ones that don't include the discrete state (unit delay)
    linPlnt.A.data(:,:,ii) = x.a(1:numStates,1:numStates);
    linPlnt.B.data(:,:,ii) = x.b(1:numStates);
    linPlnt.C.data(:,:,ii) = x.c(1:numStates,1:numStates);
    linPlnt.D.data(:,:,ii) = x.d(1:numStates);
    catch ME
        rethrow(ME)
        plotLinearPlant(linPlnt)
        fprintf();
    end
end

end