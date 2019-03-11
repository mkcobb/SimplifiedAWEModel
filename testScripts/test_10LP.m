% Script to test spatial ILC
clear;close all;clc;
SimplifiedModel_init % Initialize model

format compact
plotSwitch = true;
pathStep = 0.005; % Path discretization level used in the trajectory linearization
velocityWeight = 1; % Weight on velocity in performance index for ILC update
pathHeight_m  = 15;
pathWidth_m = 50;
waypointPositionTolerance = 0.25;
headingTol = 25*pi/180;
finalHeadingTarget = initialHeading_deg*pi/180;
numIterations = 100; % Total number of iterations (including initialization lab)
searchDistance_none = 0.05;
simulationTimeStep_s = 0.02;
waypointPathVariables = [0.25 0.5 0.75 1]; % Path variables of waypoints
ctrlInputBound = 1*pi/180; % Bounds on change in u btwn iteration
numSteps = round(1/pathStep)+1;
waypointPathIndices = round(waypointPathVariables./pathStep)+1;

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
% Set fmincon options
options = optimoptions('fmincon','MaxIterations',20000,...
    'MaxFunctionEvaluations',200000,'ConstraintTolerance',0.001,...
    'Display','off');

% Bounds on deltau (change in ff control input btwn iterations)
magnitude = ctrlInputBound;
ctrlInputBound = repmat(ctrlInputBound,[numSteps 1]);
ub =   ctrlInputBound;
lb = - ctrlInputBound;

% Initialize change in control input vector
initialGuess   = zeros(size(lb));
deltauStarPrev = initialGuess;
% initialGuess(round(0*0.125*numSteps)+1:round(1*0.125*numSteps)) = -magnitude;
% initialGuess(round(1*0.125*numSteps)+1:round(2*0.125*numSteps)) =  magnitude;
% initialGuess(round(2*0.125*numSteps)+1:round(3*0.125*numSteps)) = -magnitude;
% initialGuess(round(3*0.125*numSteps)+1:round(4*0.125*numSteps)) =  magnitude;
% initialGuess(round(4*0.125*numSteps)+1:round(5*0.125*numSteps)) = -magnitude;
% initialGuess(round(5*0.125*numSteps)+1:round(6*0.125*numSteps)) =  magnitude;
% initialGuess(round(6*0.125*numSteps)+1:round(7*0.125*numSteps)) = -magnitude;
% initialGuess(round(7*0.125*numSteps)+1:round(8*0.125*numSteps)) =  magnitude;


% PsiV matrix, J = x^T * PsiV * x
PsiV = diag(repmat([0 0 -velocityWeight 0]',[numSteps 1]));

% PsiW matrix, picks off values at waypoints
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

% initialize figures
figure;
ax1 = subplot(2,2,[1 2]);
set(gca,'FontSize',24)
set(gca,'NextPlot','add');
for ii = 1:length(waypointPathIndices) % Plot waypoint boxes
    rectangle('Position',...
        [rw(2*ii-1)-DeltaTol(2*ii-1) rw(2*ii)-DeltaTol(2*ii) 2*DeltaTol(2*ii-1) 2*DeltaTol(2*ii)],...
        'EdgeColor','r')
end
plot(squeeze(tscc{1}.xPosition_m.data),squeeze(tscc{1}.yPosition_m.data),'Color','g');
nominalPath = pathPosition(linspace(0,1,1000),pathWidth_m,pathHeight_m);
plot(nominalPath(:,1),nominalPath(:,2),'color','b');
grid on
xlabel('X Position')
ylabel('Y Position')

ax2 = subplot(2,2,3);
set(gca,'FontSize',24)
set(gca,'NextPlot','add');
grid on
xlabel('Path Variable')
ylabel('deltauStar, [deg]')

ax3 = subplot(2,2,4);
set(gca,'FontSize',24)
set(gca,'NextPlot','add');
grid on
xlabel('Iteration Number')
ylabel({'\% Duration','of First Sim'})
scatter(1,100,'CData',[0 1 0]);


% Iteration 1 was the nonlinear simulation used for intialization, start at 2
for ii = 2:numIterations
    % Linearize around the previous trajectory
    [~,linPlntDisc] = linearizePlant(tscc{ii-1},'Path',pathStep);
    
    % Now build the lifted system representation
    [F,G] = buildLiftedSytemMatrix(linPlntDisc.A.data,linPlntDisc.B.data);
    
    % Lifted state vector
    x0 = linPlntDisc.stateVector.data(:);
    
    % Difference in init cond. (next sim starts at xFinal of current sim)
    % deltax0 = tsc.stateVector.data(:,:,end)-tsc.stateVector.data(:,:,1);
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
    b = [ -rw + DeltaTol + PsiW*(x0+F*deltax0);...
        rw + DeltaTol - PsiW*(x0+F*deltax0)];
    % Add heading constraint to end of A and b
    
    %     [deltauStar,~,exitflag,~] = ...
    %         fmincon(@(x) x'*H*x + f*x,deltauStarPrev,A,b,Aeq,beq,lb,ub,[],options);
    [deltauStar,~,exitflag,~,~] = linprog(f,A,b,Aeq,beq,lb,ub);
    if isempty(deltauStar) || exitflag ~= 1% If linprog fails, just run the previous ff element
        deltauStar = deltauStarPrev;
    else
        deltauStar = deltauStar + deltauStarPrev;
    end
    
    deltauStarPrev = deltauStar;
    
    
    % Get lifted system vector from optimal control input
    xStar = x0+F*deltax0+G*deltauStar;
    xStar = reshape(xStar,[4 1 numel(linPlntDisc.A.Time)]); % Reshape to [1 4 N]
    
    % Calculate the path tracked by the feedforward element
    % (note that the fb element tracks the analytical path and
    % ff element tracks the numerical path defined here)
    ffPathBreakpoints = linPlntDisc.A.Time; % Breakpoints
    ffPathX = squeeze(xStar(1,1,:)); % X positions
    ffPathY = squeeze(xStar(2,1,:)); % Y positions
    
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
    totalSimTime =tsc.currentPathPosition_none.Time(end);
    tscc{ii} = cropTSC2LapNumber(tsc,numberOfLaps_none); % Save timeseries data
    
    % Plot things
    plot(squeeze(tsc.xPosition_m.data),squeeze(tsc.yPosition_m.data),'Parent',ax1)
    plot(linPlntDisc.stateVector.Time, (180/pi)*deltauStar,'Parent',ax2)
    if exitflag==1
        scatter(ii,100*totalSimTime/baselineDuration,'CData',[0 0 0],'Marker','o')
    else
        scatter(ii,100*totalSimTime/baselineDuration,'CData',[1 0 0],'Marker','x')
    end
end

% Plot some more things
figure
set(gca,'NextPlot','add')
set(gca,'FontSize',24)
for ii = 1:numel(tscc)
    scatter(ii,    mean(tscc{ii}.speed_mPs.data(end)),'CData',[0 0 0 ])
end
xlabel('Iteration Number')
ylabel('Mean Speed [m/s]')


save('output.mat','tscc')