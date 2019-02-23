% TEST 2: COMPARE THE RESULTS FROM THE NONLINEAR MODEL WITH THE RESULTS
% FROM THE PATH-LINEARIZED MODEL USING A SLIGHTLY PERTURBED INPUT
plotSwitch = true;


%% First run the nonlinear model to get the linearization trajectory
close all;clc
clearvars tsc tscOrig
SimplifiedModel_init
simulationDuration_s = inf;
numberOfLaps_none = 5;
VSSC_CONTROLLER = 1; % Run the pure pursuit controller
VSSC_PLANT = 1;      % Run the nonlinear, time-domain plant
fprintf('\nFinding initial trajectory')
sim('SimplifiedModel_cm')
parseLogsout;
tscOrig = cropTSC2LapNumber(tsc,1);
if plotSwitch
    plot(squeeze(tscOrig.xPosition_m.data),squeeze(tscOrig.yPosition_m.data))
    hold on
    grid on
    path = pathPosition(linspace(0,1,1000),pathWidth_m,pathHeight_m);
    plot(path(:,1),path(:,2))
    title('Original Nonlinear Simulation')
end
clearvars tsc

%% Now do the linearizations and build the lookup tables
fprintf('\nLinearizing model')
pathStep = 0.001;
linPlnt = linearizePlant(tscOrig,'Analytical','Path');

plotLinearPlant(linPlnt)
if plotSwitch
    plotLinearPlant(linPlnt);
end

%% Perturb the input and run the nonlinear model
clearvars tsc tscNLPert
VSSC_CONTROLLER = 2; % Run the open loop controller
VSSC_PLANT      = 1; % Run the nonlinear, time-domain plant
% Set the initial conditions
initialXPosition_m          = tscOrig.xPosition_m.data(1);
initialYPosition_m          = tscOrig.yPosition_m.data(1);
initialSpeed_mPs            = tscOrig.speed_mPs.data(1);
initialHeading_deg          = tscOrig.heading_rad.data(1)*180/pi;
initialPathPosition_none    = 0;

% Set the control trajectory for the open loop time-domain controller
openLoopHeadingSetpoint_rad         = tscOrig.headingSetpoint_rad;
openLoopHeadingSetpoint_rad.Time    = openLoopHeadingSetpoint_rad.Time-...
    openLoopHeadingSetpoint_rad.Time(1);

% Apply the perturbation
perturbation = nan(size(openLoopHeadingSetpoint_rad.data));
% Perturbation 1: disturb the last bit
% perturbation(:) = flip((10*pi/180)*exp(-linspace(0,10,numel(openLoopHeadingSetpoint_rad.data))));

% Perturbation 2: disturb the first bit    
perturbation(:) = (10*pi/180)*exp(-linspace(0,10,numel(openLoopHeadingSetpoint_rad.data)));
openLoopHeadingSetpoint_rad.data = ...
    openLoopHeadingSetpoint_rad.data + perturbation;    

% Set the simulation duration
simulationDuration_s = openLoopHeadingSetpoint_rad.Time(end);

fprintf('\nRunning nonlinear model on perturbed trajectory')
sim('SimplifiedModel_cm')
parseLogsout;
tscNLPert = tsc;
clearvars tsc

%% Run the linear model on the perturbed input
% This model replaces time with the path variable, so integration goes from
% 0 to 1 and marches the path position, not time.
clearvars tsc tscLnPert

VSSC_CONTROLLER = 2; % Run the open loop controller
VSSC_PLANT      = 2; % Run the linear, path-domain plant

% Overwrite the time of the open loop controller with the path position
openLoopHeadingSetpoint_rad.time = [tscOrig.currentPathPosition_none.data(1:end-1); 1] ;
% openLoopHeadingSetpoint_rad = linPlntA.ctrlInput;
% Set the simulation duration
simulationDuration_s = openLoopHeadingSetpoint_rad.Time(end);

% Set the initial conditions on deltaX (state vector)
initialXPosition_m       = 0;
initialYPosition_m       = 0;
initialSpeed_mPs         = 0;
initialHeading_deg       = 0;
initialPathPosition_none = 0;

fprintf('\nRunning linear model on perturbed trajectory')
sim('SimplifiedModel_cm')
parseLogsout;
tscLnPert = tsc;
clearvars tsc

%% Plot all the results
lineWidth = 2;
fontSize = 32;
close all
clearvars tsc
folder = fileparts(which('SimplifiedModel.prj'));
folder = fullfile(folder,'output','figures');


% Plot the perturbation
figure
plot(squeeze(perturbation)*180/pi,'LineWidth',lineWidth)
box off
set(gca,'FontSize',fontSize)
xlabel('Vector Index','FontSize',fontSize)
ylabel('Control Input Perturbation, [deg]','FontSize',fontSize)

% Plot the heading
figure;
plot(tscNLPert.currentPathPosition_none.data,...
    squeeze(tscNLPert.heading_rad.data),...
    'DisplayName','NL Model + Pert.','LineWidth',lineWidth)
grid on
hold on
plot(tscLnPert.heading_rad,...
    'DisplayName','Lin Model + Pert.','LineWidth',lineWidth)
plot(tscOrig.currentPathPosition_none.data,...
    squeeze(tscOrig.heading_rad.data),...
    'DisplayName','NL Model, No Pert','LineWidth',lineWidth)
legend
xlabel('Path Variable')
ylabel('Heading, [rad]')
set(gca,'FontSize',fontSize)
fileName = 'tst2_HeadingComparison';

savePlot(gcf,folder,fileName);

% Plot the speed
figure;
plot(tscNLPert.currentPathPosition_none.data,squeeze(tscNLPert.speed_mPs.data),...
    'DisplayName','NL Model + Pert.','LineWidth',lineWidth)
grid on
hold on
plot(tscLnPert.speed_mPs,...
    'DisplayName','Lin Model + Pert.','LineWidth',lineWidth)
plot(tscOrig.currentPathPosition_none.data,squeeze(tscOrig.speed_mPs.data),...
    'DisplayName','NL Model, No Pert','LineWidth',lineWidth)
legend
xlabel('Path Variable')
ylabel('Speed, [m/s]')
set(gca,'FontSize',fontSize)
fileName = 'tst2_SpeedComparison';
savePlot(gcf,folder,fileName);

% Plot the 2D path
figure;
plot(squeeze(tscNLPert.xPosition_m.data),squeeze(tscNLPert.yPosition_m.data),...
    'DisplayName','NL Model + Pert.','LineWidth',lineWidth)
grid on
hold on
plot(squeeze(tscLnPert.xPosition_m.data),...
    squeeze(tscLnPert.yPosition_m.data),...
    'DisplayName','Lin Model + Pert.','LineWidth',lineWidth)
plot(squeeze(tscOrig.xPosition_m.data),squeeze(tscOrig.yPosition_m.data),...
    'DisplayName','NL Model, No Pert','LineWidth',lineWidth)

legend
xlabel('x Position, [m]')
ylabel('y Position, [m]')
set(gca,'FontSize',fontSize)
fileName = 'tst2_PositionComparison';
savePlot(gcf,folder,fileName);