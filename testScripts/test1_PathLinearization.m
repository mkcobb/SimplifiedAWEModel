% TEST 1: COMPARE THE RESULTS FROM THE NONLINEAR MODEL WITH THE RESULTS
% FROM THE PATH-LINEARIZED MODEL FOR AN UNPERTURBED INPUT.  THEY SHOULD
% MATCH EXACTLY

%% First run the nonlinear model to get the linearization trajectory
close all;clc
SimplifiedModel_init
simulationDuration_s = inf;
numberOfLaps_none = 5;
sim('SimplifiedModel_cm')
parseLogsout;
tscc = cropTSC2LapNumber(tsc,numberOfLaps_none);
clearvars tsc

%% Now do the linearizations and build the lookup tables
decimation = 50;

numStates   = numel(tscc.stateVector.data(:,:,1));          % get the number of states in the model
numInputs   = numel(tscc.headingSetpoint_rad.data(:,:,1));  % Get the number of inputs

stepsInx    = 1:decimation:numel(tscc.currentPathPosition_none.data);
if stepsInx(end)~=numel(tscc.currentPathPosition_none.data)
    stepsInx = [stepsInx numel(tscc.currentPathPosition_none.data)];
end
numSteps    = numel(stepsInx);

% Set up the lookup tables that will comprise the linear path-parameterized plant
linPlnt.A = timeseries(nan([numStates numStates numSteps]),tscc.currentPathPosition_none.data(stepsInx));
linPlnt.B = timeseries(nan([numStates numInputs numSteps]),tscc.currentPathPosition_none.data(stepsInx));

linPlnt.C = linPlnt.A;
linPlnt.D = linPlnt.B;

linPlnt.stateVector       = timeseries(tscc.stateVector.data(:,:,stepsInx)          ,tscc.currentPathPosition_none.data(stepsInx));
linPlnt.ctrlInputLookup   = timeseries(tscc.headingSetpoint_rad.data(:,:,stepsInx)  ,tscc.currentPathPosition_none.data(stepsInx));

projectionStartPoint = tscc.currentPathPosition_none.data(stepsInx); % initial condition on the path projection

cnt = 1;
for ii = 1:numSteps
    initialPathPosition_none =  projectionStartPoint(ii); % remember to set the initial condition on path projection
    [linPlnt.A.data(:,:,cnt),linPlnt.B.data(:,:,cnt),...
        linPlnt.C.data(:,:,cnt),linPlnt.D.data(:,:,cnt)] =...
        linmod('linearizationPlantModel_cl_linModel',...
        linPlnt.stateVector.data(:,:,ii),... 
        linPlnt.ctrlInputLookup.data(ii));
    cnt = cnt+1
end

%% Now run the linear path-parameterized model
% This model replaces time with the path variable, so integration goes from
% 0 to 1 and marches the path position, not time.
clearvars tscLn

% Define the inputs, as functions of the path variable
command         = linPlnt.ctrlInputLookup;
linPoint        = linPlnt.stateVector;

simulationDuration_s = 1; % This is really the final value of the path variable
simulationTimeStep_s = 0.001; % This is really the path variable step size

% Set the initial conditions on deltaX (state vector)
initialXPosition_m      = 0;
initialYPosition_m      = 0;
initialSpeed_mPs        = 0;
initialHeading_deg      = 0 ;
initialPathPosition_none = 0;

sim('linearPathDomainPlnt_th')

%% Plot all the results
close all
folder = fileparts(which('SimplifiedModel.prj'));
folder = fullfile(folder,'output','figures');

% Plot the heading
figure;
plot(tscLn.heading_rad,'DisplayName','Path-Linearized Results','LineWidth',2)
grid on
hold on
plot(command,...
    'DisplayName','Nonlinear Results $\rightarrow$ Linear Command','LineWidth',2)
legend
xlabel('Path Variable')
ylabel('Heading, [rad]')
set(gca,'FontSize',24)
filename = 'tst1_HeadingComparison';
savePlot(gcf,folder,fileName);

% Plot the speed
figure;
plot(tscLn.speed_mPs,'DisplayName','Path-Linearized Results','LineWidth',2)
grid on
hold on
plot(pathVariable,squeeze(tscc.speed_mPs.data),...
    'DisplayName','Nonlinear Results','LineWidth',2)
legend
xlabel('Path Variable')
ylabel('Speed, [m/s]')
set(gca,'FontSize',24)
filename = 'tst1_SpeedComparison';
savePlot(gcf,folder,fileName);

% Plot the 2D path
figure;
plot(squeeze(tscLn.xPosition_m.data),...
    squeeze(tscLn.yPosition_m.data),...
    'DisplayName','Path Linearized Results','LineWidth',2)
grid on
hold on
plot(squeeze(tscc.xPosition_m.data),squeeze(tscc.yPosition_m.data),...
    'DisplayName','Nonlinear Model Results','LineWidth',2)
legend
xlabel('x Position, [m]')
ylabel('y Position, [m]')
set(gca,'FontSize',24)
filename = 'tst1_PositionComparison';
savePlot(gcf,folder,fileName);

% Plot the elements of A
h.fig.A = figure;
axNum = 1;
for ii = 1:size(linPlnt.A.data(:,:,1),1)
    for jj = 1:size(linPlnt.A.data(:,:,1),2)
        subplot(size(linPlnt.A.data(:,:,1),1),size(linPlnt.A.data(:,:,1),2),axNum)
        plot(linPlnt.A.Time,...
            squeeze(linPlnt.A.data(ii,jj,:)))
        hold on
        try
            plot(ALinSim.time,squeeze(ALinSim.data(ii,jj,:)))
        catch
        end
        ylabel(['A( ' num2str(ii) ' , ' num2str(jj) ' )'])
        grid on
        axNum = axNum+1;
    end
end
linkaxes(findall(gcf,'Type','axes'),'x')
set(findall(gcf,'Type','axes'),'FontSize',16)
name = 'tst1_AMatrix';
savePlot(gcf,folder,name)

% Plot the elements of B
h.fig.B = figure;
axNum = 1;
for ii = 1:size(linPlnt.B.data(:,:,1),1)
    for jj = 1:size(linPlnt.B.data(:,:,1),2)
        subplot(size(linPlnt.B.data(:,:,1),1),size(linPlnt.B.data(:,:,1),2),axNum)
        plot(linPlnt.B.Time,...
            squeeze(linPlnt.B.data(ii,jj,:)))
        hold on
        try
            plot(BLinSim.time,squeeze(BLinSim.data(ii,jj,:)))
        catch
        end
        ylabel(['B( ' num2str(ii) ' , ' num2str(jj) ' )'])
        grid on
        axNum = axNum+1;
    end
end
linkaxes(findall(gcf,'Type','axes'),'x')
set(findall(gcf,'Type','axes'),'FontSize',16)
name = 'tst1_BMatrix';
savePlot(gcf,folder,name)

% Plot the elements of deltaX obtained from simulation
h.fig.deltaXSim = figure;
axNum = 1;
for ii = 1:size(deltaXSim.data,1)
    for jj = 1:size(deltaXSim.data,2)
        subplot(size(deltaXSim.data,1),size(deltaXSim.data,2),axNum)
        plot(deltaXSim.time,...
            squeeze(deltaXSim.data(ii,jj,:)))
        ylabel(['$\delta \vec{x}$( ' num2str(ii) ' , ' num2str(jj) ' )'])
        grid on
        axNum = axNum+1;
    end
end
linkaxes(findall(gcf,'Type','axes'),'x')
set(findall(gcf,'Type','axes'),'FontSize',16)
name = 'tst1_deltaXSim';
savePlot(gcf,folder,name)


% Plot the elements of deltaU obtained from simulation
h.fig.deltaUSim = figure;
axNum = 1;
for ii = 1:size(deltaUSim.data,1)
    for jj = 1:size(deltaUSim.data,2)
        subplot(size(deltaUSim.data,1),size(deltaUSim.data,2),axNum)
        plot(deltaUSim.time,...
            squeeze(deltaUSim.data(ii,jj,:)))
        ylabel(['$\delta \vec{u}$( ' num2str(ii) ' , ' num2str(jj) ' )'])
        grid on
        axNum = axNum+1;
    end
end
linkaxes(findall(gcf,'Type','axes'),'x')
set(findall(gcf,'Type','axes'),'FontSize',16)
name = 'tst1_deltaUSim';
savePlot(gcf,folder,name)


