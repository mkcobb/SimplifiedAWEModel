nonlinearPlnt_ul_init;

simulationDuration_s = 10;
simulationTimeStep_s = 0.001;

startTime = 2;
endTime   = 5;

lowValue  = -10*pi/180;
highValue = 190*pi/180;

headingCommand_rad = timeseries;

timeVec = 0:simulationTimeStep_s:simulationDuration_s;

headingCommand_rad.Time=timeVec;

headingCommand_rad.Data=lowValue*ones(size(headingCommand_rad.Time));

headingCommand_rad.Data(and(timeVec<=endTime,timeVec>=startTime)) = highValue;
sim('nonlinearPlant_th')

plot(x.Data,y.Data)

figure
headingCommand_rad.plot
grid on
hold on
heading.plot