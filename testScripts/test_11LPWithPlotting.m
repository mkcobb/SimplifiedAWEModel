% Script to test spatial ILC
clear;close all;clc;
format compact

SimplifiedModel_init % Initialize model

pathStep       = 0.005; % Path discretization level used in the trajectory linearization
velocityWeight = 1; % Weight on velocity in performance index for ILC update
pathHeight_m   = 15;
pathWidth_m    = 50;
constraints    = 'waypointRestrictive';
finalHeadingTarget = initialHeading_deg*pi/180;
numIterations  = 50; % Total number of iterations
searchDistance_none  = 0.05;
simulationTimeStep_s = 0.02;
waypointPathVariables = [0.25 0.5 0.75 1]; % Path variables of waypoints
numSteps = round(1/pathStep)+1;
waypointPathIndices = round(waypointPathVariables./pathStep)+1;
lineWidth = 2;
markerSize = 100;
outputDir = fullfile(fileparts(which('SimplifiedModel_cm')),'output','figures');

switch constraints
    case 'balanced'
        waypointPositionTolerance = 0.5;
        ctrlInputBound = 1*pi/180; 
        headingTol     = 25*pi/180;

    case 'waypointRestrictive'
        waypointPositionTolerance = 0.5;
        ctrlInputBound = 5*pi/180; 
        headingTol     = 25*pi/180;

    case 'inputRestrictive'
        waypointPositionTolerance = 1;
        ctrlInputBound = 0.1*pi/180; 
        headingTol     = 25*pi/180;
end


%% First run the nonlinear model to get the initial trajectory
simulationDuration_s = inf;
numberOfLaps_none = 5; % Run 5 laps to allow settling
VSSC_CONTROLLER = 1; % Run the pure pursuit controller
VSSC_PLANT = 1;      % Run the nonlinear, time-domain plant
sim('SimplifiedModel_cm')
parseLogsout;
tscc{1} = cropTSC2LapNumber(tsc,numberOfLaps_none); % Crop to last lap and save data
baselineDuration = tscc{1}.currentPathPosition_none.Time(end);

%% ILC Loop
% Bounds on deltau (change in ff control input btwn iterations)
ctrlInputBound = repmat(ctrlInputBound,[numSteps 1]);
ub =   ctrlInputBound;
lb = - ctrlInputBound;

% Initialize change in control input vector
deltauStarPrev = zeros(size(lb));

% Velocity weighting matrix in performance index
PsiV = diag(repmat([0 0 -velocityWeight 0]',[numSteps 1]));

% PsiW matrix, picks off x and y values at waypoints
PsiW = cell([2*numel(waypointPathIndices) numSteps]);
PsiW(:) = {[0 0 0 0]};
for ii = 1:length(waypointPathIndices)
    PsiW(2*ii-1,waypointPathIndices(ii)) = {[1 0 0 0]};
    PsiW(2*ii  ,waypointPathIndices(ii)) = {[0 1 0 0]};
end
PsiW = cell2mat(PsiW);

% Add a line to the end so that it also picks off the heading
PsiW(end+1,:) = zeros(1,size(PsiW,2));
PsiW(end,end) = 1;

% rw "lifted" vector of waypoint x-y positions
rw = pathPosition(waypointPathVariables,pathWidth_m,pathHeight_m);
rw = reshape(rw',[numel(rw) 1]);

% Add heading to the end
rw(end+1) = finalHeadingTarget;

% Tolerance for inequality waypoint tracking ("box" around waypoints)
DeltaTol = repmat(waypointPositionTolerance*[1 1]',[numel(waypointPathVariables) 1]);
DeltaTol = [DeltaTol;headingTol];

% Set the controller to the ff+fb controller
VSSC_CONTROLLER = 5; % Run the ILC pure pursuit controller
numberOfLaps_none = 1;

initializeFigs

%% Iteration 1 was the nonlinear simulation used for intialization, start at 2
for ii = 2:numIterations
    %% LINEARIZE
    [~,linPlntDisc] = linearizePlant(tscc{ii-1},'Path',pathStep);
    
    %% LIFT
    [F,G] = buildLiftedSytemMatrix(linPlntDisc.A.data,linPlntDisc.B.data);
    
    %% LP UPDATE
    % Lifted state vector
    x0 = linPlntDisc.stateVector.data(:);
    
    % Difference in init cond. (next sim starts at xFinal of current sim)
    deltax0 = zeros(size(tsc.stateVector.data(:,:,end)));
    
    % Build H and f
    H = G'*PsiV*G;
    H = round(H,10); % Round to 10 decimal places to ensure symmetry of hessian
    %     f = 2*(x0+F*deltax0)'*PsiV*G;
    f = ones([1,size(G,1)])*PsiV*G;
    
    % Build Aeq and beq
    Aeq = [];
    beq = [];
    
    % A and b (inequality constraints)
    A = [ -PsiW*G;...
        PsiW*G];
    b = [ -rw + DeltaTol + PsiW*(x0+F*deltax0);... % Lower limits
           rw + DeltaTol - PsiW*(x0+F*deltax0)];   % Upper limits
    
    % Economic ILC LP update step
    [deltauStar,~,exitflag,~,lambda] = linprog(f,A,b,Aeq,beq,lb,ub);

    % If the optimization had no feasible solutions, set update to zero
    if isempty(deltauStar) || exitflag ~= 1
        deltauStar = zeros(size(deltauStarPrev));
    end
        
    % Get lifted system vector from optimal control input
    xStar{ii} = x0+F*deltax0+G*deltauStar;
    xStar{ii} = reshape(xStar{ii},[4 1 numel(linPlntDisc.A.Time)]); % Reshape to [1 4 N]
    
    deltauStar = deltauStar + deltauStarPrev;
    deltauStarPrev = deltauStar;
    
    %% NEXT LAP
    % Calculate the path tracked by the feedforward element
    % (note that the fb element tracks the analytical path and
    % ff element tracks the numerical path defined here)
    ffPathBreakpoints = linPlntDisc.A.Time; % Breakpoints
    ffPathX = squeeze(xStar{ii}(1,1,:)); % X positions
    ffPathY = squeeze(xStar{ii}(2,1,:)); % Y positions
    
    % Run a nonlinear simulation
    % Set the initial conditions
    initialXPosition_m  = tscc{1}.stateVector.data(1,:,1);
    initialYPosition_m  = tscc{1}.stateVector.data(2,:,1);
    initialSpeed_mPs    = tscc{1}.stateVector.data(3,:,1);
    initialHeading_deg  = tscc{1}.stateVector.data(4,:,1)*180/pi;
    initialPathPosition_none = tscc{1}.currentPathPosition_none.data(1);
    
    % Run the simulation
    sim('SimplifiedModel_cm')
    parseLogsout;
    totalSimTime = tsc.currentPathPosition_none.Time(end);
    tscc{ii}     = cropTSC2LapNumber(tsc,numberOfLaps_none); % Save timeseries data
    
    updateFigs
    
end


