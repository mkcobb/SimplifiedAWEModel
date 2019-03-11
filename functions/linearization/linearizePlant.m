function [linPlnt,linPlntDisc] = linearizePlant(tsc,type,stepSize)
% Function to calculate the path-linearized version of a plant
% function returns the struct linPlnt which contains elements A, B, C, D,
% stateVector, ctrlInput.  These are all timeseries where the "time" is
% actually the path variable.


%% Preallocate a bunch of timeseries into a structure to contain the results
% in these timeseries "time" will correspond to the quantity that the user
% has specified to parameterize the trajectory.  If the user specified
% "path" for the method argument, then time will actually be the path
% variable.
numStates   = numel(tsc.stateVector.data(:,:,1));          % get the number of states in the model
numInputs   = 1;  % Get the number of inputs

% Create a "time" timeseries and add it to the collection to keep
% track of the time/path variable relationship through all the following
% resamples, etc.
switch lower(type)
    case 'path'
        % Step 1: Overwrite "time" field of all timeseries in tsc with path 
        % variable and re-sample to specified time step
        
        % Normalize the old time vector so that it goes 0-1 inclusive
        oldTimeVec = tsc.currentPathPosition_none.data;
        oldTimeVec = oldTimeVec-oldTimeVec(1);
        oldTimeVec = oldTimeVec./oldTimeVec(end);
        newTimeVec = 0:stepSize:1;
        fieldNames = fields(tsc);
        for ii = 1:numel(fieldNames)
            tsc.(fieldNames{ii}).Time = oldTimeVec;
            tsc.(fieldNames{ii}) = resample(tsc.(fieldNames{ii}),newTimeVec);
        end
        % Step 2: Preallocate
        % Set up the lookup tables for the state vector and control input
        % Get the points in state space to linearize around
        % Set the "time" to be the path position in the timeseries
        linPlnt.stateVector = tsc.stateVector;
        % Get the control inputs to linearize around
        % Set the "time" to be the path position in the timeseries
        linPlnt.ctrlInput   = tsc.headingSetpoint_rad;
               
        numSteps    = numel(newTimeVec);
        
        % Set up the lookup tables that will comprise the linear path-parameterized plant
        % Set the "time" to be the path position in the timeseries
        linPlnt.A = timeseries(...
            nan([numStates numStates numSteps]),...
            newTimeVec);
        linPlnt.B = timeseries(...
            nan([numStates numInputs numSteps]),...
            newTimeVec);
        % Step 3: Do the path-parameterized linearization
        analyticalPathLinearization
 
    case 'time'
        % Step 1: re-sample to specified time step
        newTimeVec = 0:stepSize:tsc.currentPathPosition_none.Time(end)-tsc.currentPathPosition_none.Time(1);
        fieldNames = fields(tsc);
        for ii = 1:numel(fieldNames) % For each timeseries in the collection
            % Shift time of all timeseries so it starts at zero
            tsc.(fieldNames{ii}).Time = tsc.(fieldNames{ii}).Time- tsc.(fieldNames{ii}).Time(1);
            % Resample to new time vector
            tsc.(fieldNames{ii}) = resample(tsc.(fieldNames{ii}),newTimeVec);
        end
        
        % Step 2: Preallocate data fields
        numSteps    = numel(newTimeVec);
        % Get the state vector to linearize around
        linPlnt.stateVector = tsc.stateVector;
        % Get the control inputs to linearize around
        linPlnt.ctrlInput   = tsc.headingSetpoint_rad;
        % Add the path variable to the output
        linPlnt.pathVariable = tsc.currentPathPosition_none;
        % Build timeseries for A and B matrices
        linPlnt.A = timeseries(...
            nan([numStates numStates numSteps]),...
            newTimeVec);
        linPlnt.B = timeseries(...
            nan([numStates numInputs numSteps]),...
            newTimeVec);
        
        % Step 3: Do the linearization
        analyticalTimeLinearization
end

linPlntDisc = continuousToDiscrete(linPlnt,stepSize);

end