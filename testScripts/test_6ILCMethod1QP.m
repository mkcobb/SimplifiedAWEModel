% Script to test spatial ILC
clear;close all;clc;
SimplifiedModel_init

plotSwitch = true;
pathStep = 0.005; % Path discretization level used in the trajectory linearization
velocityWeight = 1; % Weight on velocity in performance index for ILC update
waypointWeight = 1;
numIterations = 50;
searchDistance_none = 0.05;
simulationTimeStep_s = 0.02;
waypointPathVariables = [0.25 0.75 1];
ctrlInputBound = 5*pi/180;
numSteps = round(1/pathStep)+1;
waypointPathIndices = round(waypointPathVariables./pathStep)+1;

%% First run the nonlinear model to get the initial trajectory
simulationDuration_s = inf;
numberOfLaps_none = 5;
VSSC_CONTROLLER = 1; % Run the pure pursuit controller
VSSC_PLANT = 1;      % Run the nonlinear, time-domain plant
sim('SimplifiedModel_cm')
parseLogsout;
tscc{1} = cropTSC2LapNumber(tsc,numberOfLaps_none);

%% ILC Loop
% Bounds on deltau
ctrlInputBound = repmat(ctrlInputBound,[numSteps 1]);
ub =   ctrlInputBound;
lb = - ctrlInputBound;
% Option 1: Quadratic program
options = optimoptions('quadprog','Algorithm','interior-point-convex','ConstraintTolerance',0.1);

% Option 2: Fmincon
% options = optimoptions('fmincon','MaxIterations',10000,'MaxFunctionEvaluations',10000);

% PsiV, weighting matrix to pick off velocity state
PsiV = diag(repmat([0 0 -velocityWeight 0]',[numSteps 1]));

% PsiW, matrix to pick off the values of x and y at the spacific path
% positions
PsiW = cell([2*numel(waypointPathIndices),numSteps]);
PsiW(:) = {[0 0 0 0]};
for ii = 1: numel(waypointPathIndices)
   PsiW{2*ii-1,waypointPathIndices(ii)} =  [1 0 0 0];
   PsiW{2*ii  ,waypointPathIndices(ii)} =  [0 1 0 0];
end
PsiW = cell2mat(PsiW);
rw = pathPosition(waypointPathVariables,pathWidth_m,pathHeight_m);
rw = reshape(rw',[2*numel(waypointPathVariables),1]);

% Set the controller to the open loop, path domain controller
VSSC_CONTROLLER = 5; % Run the ILC pure pursuit controller
numberOfLaps_none = 1;

figure;
ax1 = subplot(2,1,1);
set(gca,'FontSize',24)
set(gca,'NextPlot','add');
grid on

ax2 = subplot(2,1,2);
set(gca,'FontSize',24)
set(gca,'NextPlot','add');
grid on

deltauStar = zeros(size(lb));

% Iteration 1 was the nonlinear simulation used for intialization, start at 2
for ii = 2:numIterations
    % Linearize around the previous trajectory
    [~,linPlntDisc] = linearizePlant(tscc{ii-1},'Path',pathStep);
%     [linPlnt,~] = linearizePlant(tscc{ii-1},'Time',simulationTimeStep_s);
    
    % Now build the lifted system representation
    [F,G] = buildLiftedSytemMatrix(linPlntDisc.A.data,linPlntDisc.B.data);
    
    
    % ILC Update function in a perfect world
    % Inputs: nominal trajectories (x0 and u0), F, G, appropriate weighting
    % matrices (depending on whether we do quad or lin prog), limits on
    % u, deltau and deltaTimeSteps
    % Outputs: uStar
    
    % Lifted state vector
    x0 = linPlntDisc.stateVector.data(:);
    
    % Difference in init cond. (next sim starts at xFinal of current sim)
    % deltax0 = tsc.stateVector.data(:,:,end)-tsc.stateVector.data(:,:,1);
    deltax0 = zeros(size(tsc.stateVector.data(:,:,end)));
    
    % Build H and f
    H = G'*(PsiV)*G;
    H = round(H,10); % Round to 10 decimal places to ensure symmetry of hessian
    f = 2*(x0+F*deltax0)'*PsiV*G;

    % Build Aeq and beq
    Aeq = PsiW*G;
    beq = rw - PsiW*(x0+F*deltax0);
    % Option 1: quadprog
    
    deltauStar= quadprog(H,f,[],[],Aeq,beq,lb,ub,[],options); % quadprog solution
    
    xStar = x0+F*deltax0+G*deltauStar;
    xStar = reshape(xStar,[4 1 numel(linPlntDisc.A.Time)]);
    
    % Calculate the path tracked by the feedforward element (note that the
    % fb element tracks the analytical path
    ffPathBreakpoints = linPlntDisc.A.Time;
    ffPathX = squeeze(xStar(1,1,:));
    ffPathY = squeeze(xStar(2,1,:));
    
    % Run a nonlinear simulation
    % Set the initial conditions to be the values at the end of the
    % previour sim
%     initialXPosition_m  = tsc.stateVector.data(1,:,end);
%     initialYPosition_m  = tsc.stateVector.data(2,:,end);
%     initialSpeed_mPs    = tsc.stateVector.data(3,:,end);
%     initialHeading_deg  = tsc.stateVector.data(4,:,end)*180/pi;
%     initialPathPosition_none = tsc.currentPathPosition_none.data(end);
    
     % Reset the initial conditions
    initialXPosition_m  = tsc.stateVector.data(1,:,1);
    initialYPosition_m  = tsc.stateVector.data(2,:,1);
    initialSpeed_mPs    = tsc.stateVector.data(3,:,1);
    initialHeading_deg  = tsc.stateVector.data(4,:,1)*180/pi;
    initialPathPosition_none = tsc.currentPathPosition_none.data(1);
    
    sim('SimplifiedModel_cm')
    parseLogsout;
    tscc{ii} = cropTSC2LapNumber(tsc,numberOfLaps_none);
    
    plot(squeeze(tsc.xPosition_m.data),squeeze(tsc.yPosition_m.data),'Parent',ax1)
    plot(linPlntDisc.ctrlInput.Time,deltauStar,'Parent',ax2)
end
% textMe('Simulation complete')

format compact
for ii = 1:numel(tscc)
    tscc{ii}.currentPathPosition_none.Time(end)
end
for ii = 1:numel(tscc)
    mean(tscc{ii}.speed_mPs.data(end))
end


