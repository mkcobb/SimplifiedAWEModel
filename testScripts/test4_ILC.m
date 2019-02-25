% Script to test spatial ILC
clear;close all;clc;
SimplifiedModel_init

plotSwitch = true;
pathStep = 0.005; % Path discretization level used in the trajectory linearization

velocityWeight = 1; % Weight on velocity in performance index for ILC update
waypointWeight = 0;

numIterations = 30;
searchDistance_none = 0.05;
simulationTimeStep_s = 0.02;
waypointPathVariables = [0.25 0.75 1];

ctrlInputBound = 5*pi/180;
numSteps = round(1/pathStep)+1;

%% First run the nonlinear model to get the initial trajectory
simulationDuration_s = inf;
numberOfLaps_none = 1;
VSSC_CONTROLLER = 1; % Run the pure pursuit controller
VSSC_PLANT = 1;      % Run the nonlinear, time-domain plant
sim('SimplifiedModel_cm')
parseLogsout;
tscc{1} = cropTSC2LapNumber(tsc,numberOfLaps_none);

iterationDuration(1) = tsc.currentPathPosition_none.Time(end);

%% ILC Loop
% turn into column vector
ctrlInputBound = repmat(ctrlInputBound,[numSteps 1]);
options = optimoptions('quadprog','Algorithm','trust-region-reflective','Display','off');
options = optimoptions('quadprog','Algorithm','interior-point-convex');
% Build the PsiV (velocity weighting) and PsiT (terminal state) weighting
% matrices

PsiV = diag(repmat([0 0 -velocityWeight 0]',[numSteps 1]));
PsiW = cell(numSteps,numSteps);
PsiW(:) = {zeros(4,4)};

xw = cell(numSteps,1);
xw(:) = {[0 0 0 0]'};

for ii = 1:length(waypointPathVariables)
    pathIndex = round(waypointPathVariables(ii)/pathStep);
    PsiW(pathIndex,pathIndex) = {diag([waypointWeight waypointWeight 0 0])};
    xw(pathIndex) = {[pathPosition(waypointPathVariables(1),pathWidth_m,pathHeight_m) 0 0 ]'};
end
PsiW = cell2mat(PsiW);
xw = cell2mat(xw);

DeltaTol = 1*ones(6,1);
% DeltaTol = [1 0.1 1 0.1 1 0.1]';
r = [25 0 -25 0 0 0]';

PsiT = cell(numel(DeltaTol),numSteps);
PsiT(:) = {[0 0 0 0]};
cnt = 1;
for ii = 1:2:numel(DeltaTol)
    index = round(waypointPathVariables(cnt)./pathStep)+1;
    PsiT(ii,index) = {[1 0 0 0]};
    PsiT(ii+1,index) = {[0 1 0 0]};
    cnt = cnt+1;
end
PsiT = cell2mat(PsiT);
% PsiT(end-4:end,end-10:end)
% Build ub and lb vectors for deltau
ub =    ctrlInputBound;
lb =  - ctrlInputBound;

% Note: don't want to use A and b because that might be so restrictive
% that there are no feasible solutions
A = [];
b = [];


% Set the controller to the open loop, path domain controller
VSSC_CONTROLLER = 5; % Run the ILC pure pursuit controller

numberOfLaps_none = 1;

fig1 = figure;
ax1 = gca;
set(ax1,'NextPlot','add')
grid on
legend
set(gca,'FontSize',24)

fig2 = figure;
ax2 = gca;
grid on
set(ax2,'NextPlot','add')
legend
set(gca,'FontSize',24)

fig3= figure;
ax3 = gca;
grid on
set(ax3,'NextPlot','add')
legend
set(gca,'FontSize',24)
    
    
deltauStar = zeros(size(lb));
% Iteration 1 was the nonlinear simulation used for intialization, start at 2
for ii = 2:numIterations
    % Linearize around the previous trajectory
    [~,linPlntDisc] = linearizePlant(tscc{ii-1},'Analytical','Path',pathStep);
    
    % Now build the lifted system representation
    [F,G] = buildLiftedSytemMatrix(linPlntDisc.A.data,linPlntDisc.A.data);
    
    
    % ILC Update function in a perfect world
    % Inputs: nominal trajectories (x0 and u0), F, G, appropriate weighting
    % matrices (depending on whether we do quad or lin prog), limits on
    % u, deltau and deltaTimeSteps
    % Outputs: uStar
    
    % Build H and f
    H = G'*(PsiW+PsiW)*G;
    H = round(H,10); % Round to 10 decimal places to ensure symmetry of hessian
    xiLast = [tsc.xPosition_m.data(1) tsc.yPosition_m.data(1) tsc.speed_mPs.data(1) tsc.heading_rad.data(1)]';
    xiNext = [tsc.xPosition_m.data(end) tsc.yPosition_m.data(end) tsc.speed_mPs.data(end) tsc.heading_rad.data(end)]';
    deltax0 = xiNext - xiLast;
    x0 = linPlntDisc.stateVector.data(:);
    f = 2*((x0+F*deltax0)'*(PsiV+PsiW)-xw'*PsiW)*G;
    
    A = [PsiT*G;...
        -PsiT*G];
    b = [r + DeltaTol - PsiT*(x0+F*deltax0);...
        -r + DeltaTol + PsiT*(x0+F*deltax0)];
    
    Aeq = [];
    beq = [];
    
    deltauStar = quadprog(H,f,A,b,Aeq,beq,lb,ub,deltauStar,options);
    A*deltauStar
    plot(deltauStar,'Parent',ax3)
    
    xStar = x0+F*deltax0+G*deltauStar;
    %     J(ii) =   xStar'*PsiV*xStar+  (xStar-xw)'*PsiW*(xStar-xw)
    %
    xStar = reshape(xStar,[4 1 numel(linPlntDisc.A.Time)]);
    
    figure;    plot(squeeze(xStar(1,:,:)),squeeze(xStar(2,:,:)))
    
    projectionBreakpoints = linPlntDisc.A.Time;
%         p = pathPosition(projectionBreakpoints',pathWidth_m,pathHeight_m);
%         projectionX = p(:,1);
%         projectionY = p(:,2);
    %
    projectionX = squeeze(xStar(1,1,:));
    projectionY = squeeze(xStar(2,1,:));
    
    % Run a nonlinear simulation
    % Set the initial conditions
    initialXPosition_m  = xiNext(1);
    initialYPosition_m  = xiNext(2);
    initialSpeed_mPs    = xiNext(3);
    initialHeading_deg  = xiNext(4)*180/pi;
    initialPathPosition_none = tsc.currentPathPosition_none.data(end);
    
    sim('SimplifiedModel_cm')
    parseLogsout;
    tscc{ii} = cropTSC2LapNumber(tsc,numberOfLaps_none);
    
    plot(tsc.headingSetpoint_rad,'DisplayName',sprintf('Iteration %d',ii),'Parent',ax1)
    plot(squeeze(tsc.xPosition_m.data),squeeze(tsc.yPosition_m.data),'DisplayName',sprintf('Iteration %d',ii),'Parent',ax2)
    
    iterationDuration(ii) = tsc.currentPathPosition_none.time(end);
    
end
% textMe('Simulation complete')

format compact
for ii = 1:numel(tscc)
    tscc{ii}.currentPathPosition_none.Time(end)
end
for ii = 1:numel(tscc)
    mean(tscc{ii}.speed_mPs.data(end))
end


