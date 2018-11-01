startTime = 2;
endTime   = 5;

lowValue  = -10*pi/180;
highValue = 190*pi/180;

headingCommand_rad = timeseries;

timeVec = 0:simulationTimeStep_s:simulationDuration_s;

headingCommand_rad.Time=timeVec;

headingCommand_rad.Data=lowValue*ones(size(headingCommand_rad.Time));

headingCommand_rad.Data(and(timeVec<=endTime,timeVec>=startTime)) = highValue;

clearvars startTime endTime lowValue highValue timeVec