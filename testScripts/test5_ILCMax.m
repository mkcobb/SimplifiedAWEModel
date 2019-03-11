% Script for Max to test ILC
% MC created the initial version of this by cleaning/stripping down
% test4_ILC.m but the basic order of operations should be the same.
clear;close all;clc;
SimplifiedModel_init

pathStep = 0.005; % Path discretization level used in the trajectory linearization
numIterations = 30; % Number of ILC iterations in the whole simulation
searchDistance_none = 0.05; % distance ahead/behind the last known path position to search for the new path position
simulationTimeStep_s = 0.02; % simulation time step
waypointPathVariables = .05:0.05:1;

%Defining parameters for the LP and QP (Pareto) optimization steps
deltaDeltaUMax = 0.06;
deltaTMin = .98;
deltaTMax = 1.02;
deltaUMax = 2;
deltaUMin = -deltaUMax;
trackingErrorTolerance = .02;
ILCWeights.trackingCost = 1;
ILCWeights.inputCost = 1e-6;

waypointPathIndices = round(waypointPathVariables./pathStep)+1;
pathOnlyWaypoints = zeros(1/pathStep+1,2);
pathOnlyWaypoints(waypointPathIndices,:) = pathPosition(waypointPathVariables,pathWidth_m,pathHeight_m);
pathCompleteVariables = 0:pathStep:1;
pathComplete = pathPosition(pathCompleteVariables,pathWidth_m,pathHeight_m);
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

% Put this in an init script
deltauStar = zeros(floor(1/pathStep)+1,1);

% Iteration 1 was the nonlinear simulation used for intialization, start at 2
for ii = 2:numIterations
    % Step 1: Linearize (and discretize) around the previous trajectory
    [~,linPlntDiscPath] = linearizePlant(tscc{ii-1},'Path',pathStep);
    [linPlntTime,~] = linearizePlant(tscc{ii-1},'Time',pathStep);
    % Step 2: Build the lifted system representation. Here, alphaMat = G
    [F,alphaMat] = buildLiftedSytemMatrix(linPlntDiscPath.A.data,linPlntDiscPath.B.data);
    
    
    %Obtaining the corresponding timestamps at at each pathstep
    pathVariableVSTime.pathVariable = linPlntTime.pathVariable;
    pathStepTimes = interp1([pathVariableVSTime.pathVariable.Data(1:end-1);1],pathVariableVSTime.pathVariable.Time,0:pathStep:1);
    
    %Determining the A and B matrices at every pathstep
    tsin = linPlntTime.A;
    pathStepTimes(1) = 0;
    AContTimeseries = resample(tsin,pathStepTimes);
    tsin = linPlntTime.B;
    BContTimeseries = resample(tsin,pathStepTimes);
    ACont = AContTimeseries.Data;
    BCont = BContTimeseries.Data;

    
%     tsin = timeseries(pathVariableVSTime.pathVariable.Time,[pathVariableVSTime.pathVariable.Data(1:end-1);1]');
%     tsout = getsampleusingtime(tsin,0,1);
%     tsout.Data
    
    % Step 3: ILC Update
    % Inputs: nominal trajectories (x0 and u0), F, G, appropriate weighting
    % matrices (depending on whether we do quad or lin prog), limits on
    % u, deltau and deltaTimeSteps
    % Outputs: uStar, deviation from nominal trajectory
    % uStar = ILCUPDATEFUNCTIONHERE(INPUTS);
    
    
    
    %Lifted state vector from the previous iteration
    xNominal = linPlntDiscPath.stateVector.data(:);
    % Difference in init cond. (next sim starts at xFinal of current sim)
    deltax0 = tsc.stateVector.data(:,:,end)-tsc.stateVector.data(:,:,1);
    
    % A reference vector where the only inforamtion is the waypoint
    % locations
    stateReference = reshape([pathOnlyWaypoints, zeros(size(pathOnlyWaypoints,1),2)]',[4*size(pathOnlyWaypoints,1), 1]);
    % Running a spatial pareto ILC step
    deltauStar = runParetoILCStep(xNominal,deltax0,F,alphaMat,stateReference,...
        waypointPathIndices,ILCWeights.trackingCost,ILCWeights.inputCost,deltaUMin,deltaUMax);
    % Defining the gamma matrix. The gamma matrix comes from the linearization of the lifted system input-state relationship
    % which gives x = xNominal+F*deltax0+alphaMat*deltaU+gammaMat*deltaT
    gammaMat = makeGammaMatrix(linPlntDiscPath.A.Data,linPlntDiscPath.B.Data,ACont, BCont, pathStepTimes,deltauStar);
    
    %The timestepping scheme (ie, a vector with elements that say how long
    %it takes to go from one path step to the next
    TimeStepsNominal = [diff(pathStepTimes)';1]; %This last timestep is completely arbitrary and should be updated. I'm including it here or else we will get an indexing error
    % Running the time optimization step
    [deltauStar,deltaTStar] = runLPStep(xNominal,deltax0,F,alphaMat, gammaMat,stateReference,waypointPathIndices,trackingErrorTolerance,deltaDeltaUMax, deltaTMin,deltaTMax,deltauStar, TimeStepsNominal);
    
    xStar = xNominal+F*deltax0+alphaMat*deltauStar+gammaMat*deltaTStar;
    xStar = reshape(xStar,[4 1 numel(linPlntDiscPath.A.Time)]);
    
    linPlntDisc = linPlntDiscPath;
    ffPathBreakpoints = linPlntDiscPath.A.Time;
    ffPathX = squeeze(xStar(1,1,:));
    ffPathY = squeeze(xStar(2,1,:));
    % p = pathPosition(projectionBreakpoints',pathWidth_m,pathHeight_m);diff(pathStepTimes)
    % projectionX = p(:,1);
    % projectionY = p(:,2);

    % Step 4: Run a nonlinear simulation
    % Set the initial conditions
%     initialXPosition_m  = xiNext(1);
%     initialYPosition_m  = xiNext(2);
%     initialSpeed_mPs    = xiNext(3);
%     initialHeading_deg  = xiNext(4)*180/pi;
    initialXPosition_m  = tsc.stateVector.data(1,:,end);
    initialYPosition_m  = tsc.stateVector.data(2,:,end);
    initialSpeed_mPs    = tsc.stateVector.data(3,:,end);
    initialHeading_deg  = tsc.stateVector.data(4,:,end)*180/pi;
    initialPathPosition_none = tsc.currentPathPosition_none.data(end);
    
    % Run the simulation
    sim('SimplifiedModel_cm')
    % Cleanup/format output
    parseLogsout;
    tscc{ii} = cropTSC2LapNumber(tsc,numberOfLaps_none); % Crop and store into structure
    

end
plot(squeeze(tscc{1,ii}.targetX_m.Data),squeeze(tscc{1,ii}.targetY_m.Data))
hold on
plot(xNominal(1:4:end),xNominal(2:4:end))
