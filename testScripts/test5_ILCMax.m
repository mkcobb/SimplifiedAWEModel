% Script for Max to test ILC
% MC created the initial version of this by cleaning/stripping down
% test4_ILC.m but the basic order of operations should be the same.
clear;close all;clc;
SimplifiedModel_init

pathStep = 0.005; % Path discretization level used in the trajectory linearization
numIterations = 30; % Number of ILC iterations in the whole simulation
searchDistance_none = 0.05; % distance ahead/behind the last known path position to search for the new path position
simulationTimeStep_s = 0.02; % simulation time step

%% First run the nonlinear model to get the initial trajectory
simulationDuration_s = inf;
numberOfLaps_none = 1; % Number of laps to run
VSSC_CONTROLLER = 1; % Run the pure pursuit controller
VSSC_PLANT = 1;      % Run the nonlinear, time-domain plant
sim('SimplifiedModel_cm')
parseLogsout; % Create tsc timeseries collection in workspace
tscc{1} = cropTSC2LapNumber(tsc,numberOfLaps_none); % Crop to the last lap and save into a structure

%% ILC Loop
VSSC_CONTROLLER = 5; % Run the ILC + pure pursuit controller
numberOfLaps_none = 1;

% Iteration 1 was the nonlinear simulation used for intialization, start at 2
for ii = 2:numIterations
    % Step 1: Linearize (and discretize) around the previous trajectory
      [~,linPlntDisc] = linearizePlant(tscc{ii-1},'Path',pathStep);
    
    % Step 2: Build the lifted system representation
    [F,G] = buildLiftedSytemMatrix(linPlntDisc.A.data,linPlntDisc.B.data);
    
    % Step 3: ILC Update
    % Inputs: nominal trajectories (x0 and u0), F, G, appropriate weighting
    % matrices (depending on whether we do quad or lin prog), limits on
    % u, deltau and deltaTimeSteps
    % Outputs: uStar, deviation from nominal trajectory
    % uStar = ILCUPDATEFUNCTIONHERE(INPUTS);
    
    projectionBreakpoints = linPlntDisc.A.Time;
    projectionX = squeeze(xStar(1,1,:));
    projectionY = squeeze(xStar(2,1,:));
    % p = pathPosition(projectionBreakpoints',pathWidth_m,pathHeight_m);
    % projectionX = p(:,1);
    % projectionY = p(:,2);

    % Step 4: Run a nonlinear simulation
    % Set the initial conditions
    initialXPosition_m  = xiNext(1);
    initialYPosition_m  = xiNext(2);
    initialSpeed_mPs    = xiNext(3);
    initialHeading_deg  = xiNext(4)*180/pi;
    initialPathPosition_none = tsc.currentPathPosition_none.data(end);
    % Run the simulation
    sim('SimplifiedModel_cm')
    % Cleanup/format output
    parseLogsout;
    tscc{ii} = cropTSC2LapNumber(tsc,numberOfLaps_none); % Crop and store into structure
end