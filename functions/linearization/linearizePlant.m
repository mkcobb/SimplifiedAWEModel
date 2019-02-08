function linPlnt = linearizePlant(tsc,method,type,varargin)
% Function to calculate the path-linearized version of a plant
% function returns the struct linPlnt which contains elements A, B, C, D,
% stateVector, ctrlInput.  These are all timeseries where the "time" is
% actually the path variable.

%% Parse inputs
p = inputParser;
addRequired(p,'tsc'   ,@isstruct)
addRequired(p,'method',@(x) any(validatestring(x,{'Analytical','Numerical'})))
addRequired(p,'type'  ,@(x) any(validatestring(x,{'Path','Time'})))
addOptional(p,'Decimation',1,@isnumeric)
parse(p,tsc,method,type,varargin{:})

%% Preallocate a bunch of timeseries into a structure to contain the results
% in these timeseries "time" will correspond to the quantity that the user
% has specified to parameterize the trajectory.  If the user specified
% "path" for the method argument, then time will actually be the path
% variable.
numStates   = numel(tsc.stateVector.data(:,:,1));          % get the number of states in the model
numInputs   = numel(tsc.headingSetpoint_rad.data(:,:,1));  % Get the number of inputs

% Get the indices of the points to linearize around
stepsInx    = 1:p.Results.Decimation:numel(tsc.currentPathPosition_none.time);
% Include the last point available
if stepsInx(end)~=numel(tsc.currentPathPosition_none.time)
    stepsInx = [stepsInx numel(tsc.currentPathPosition_none.time)];
end
numSteps    = numel(stepsInx);

switch upper(p.Results.type)
    case 'PATH'
        parameterizationVariable = tsc.currentPathPosition_none.data(stepsInx);
    case 'TIME'
        parameterizationVariable = tsc.currentPathPosition_none.time(stepsInx);
end

% Set up the lookup tables that will comprise the linear path-parameterized plant
% Set the "time" to be the path position in the timeseries
linPlnt.A = timeseries(...
    nan([numStates numStates numSteps]),...
    parameterizationVariable);
linPlnt.B = timeseries(...
    nan([numStates numInputs numSteps]),...
    parameterizationVariable);

% Set up the lookup tables for the state vector and control input
% Get the points in state space to linearize around
% Set the "time" to be the path position in the timeseries
linPlnt.stateVector = timeseries(...
    tsc.stateVector.data(:,:,stepsInx),...
    parameterizationVariable);
% Get the control inputs to linearize around
% Set the "time" to be the path position in the timeseries
linPlnt.ctrlInput   = timeseries(...
    tsc.headingSetpoint_rad.data(:,:,stepsInx),...
    parameterizationVariable);

switch lower(p.Results.type)
    case 'path'
        switch lower(p.Results.method)
            case 'numerical'
                numericalPathLinearization
            case 'analytical'
                analyticalPathLinearization
        end
    case 'time'
        switch lower(p.Results.method)
            case 'numerical'
                numericalTimeLinearization
            case 'analytical'
                analyticalTimeLinearization
        end
end

end