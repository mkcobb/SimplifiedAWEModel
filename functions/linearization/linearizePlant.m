function [linPlnt,linPlntDisc] = linearizePlant(tsc,method,type,stepSize)
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

switch upper(type)
    case 'PATH'
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
        
%         linPlnt.C = linPlnt.A;
%         linPlnt.D = linPlnt.B;
    case 'TIME'
       
        x = 1;
end



switch lower(type)
    case 'path'
        switch lower(method)
            case 'numerical'
                numericalPathLinearization
            case 'analytical'
                analyticalPathLinearization
        end
    case 'time'
        switch lower(method)
            case 'numerical'
                numericalTimeLinearization
            case 'analytical'
                analyticalTimeLinearization
        end
end
linPlntDisc = continuousToDiscrete(linPlnt,stepSize);
end