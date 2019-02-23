% Script to test spatial ILC
clear;close all;clc;

plotSwitch = true;

%% First run the nonlinear model to get the linearization trajectory
close all;clc
clearvars tsc tscOrig
SimplifiedModel_init
simulationDuration_s = inf;
numberOfLaps_none = 1;
VSSC_CONTROLLER = 1; % Run the pure pursuit controller
VSSC_PLANT = 1;      % Run the nonlinear, time-domain plant
fprintf('\nFinding initial trajectory')
sim('SimplifiedModel_cm')
parseLogsout;
tscOrig = cropTSC2LapNumber(tsc,numberOfLaps_none);
if plotSwitch
    figure
    plot(squeeze(tscOrig.xPosition_m.data),squeeze(tscOrig.yPosition_m.data),...
        'DisplayName','Achieved Path')
    hold on
    grid on
    path = pathPosition(linspace(0,1,1000),pathWidth_m,pathHeight_m);
    plot(path(:,1),path(:,2),'DisplayName','NominalPath')
    legend
    set(gca,'FontSize',24)
    title('Original Nonlinear Simulation')
end
clearvars tsc

%% Now do the linearizations and build the lookup tables
fprintf('\nLinearizing model')
pathStep = 0.001;
[linPlnt,linPlntDisc] = linearizePlant(tscOrig,'Analytical','Path',pathStep);
if plotSwitch
    plotLinearPlant(linPlnt);
    plotLinearPlant(linPlntDisc);
end

%% Run the linear simulink model on a slightly perturbed input
clearvars tsc tscLnPert

VSSC_CONTROLLER = 2; % Run the open loop controller
VSSC_PLANT      = 2; % Run the linear, path-domain plant

% get the control trajectory
openLoopHeadingSetpoint_rad = tscOrig.headingSetpoint_rad;
openLoopHeadingSetpoint_rad.time = [tscOrig.currentPathPosition_none.data(1:end-1); 1] ;

% Apply the perturbation
perturbation = nan(size(openLoopHeadingSetpoint_rad.data));
perturbation(:) = (10*pi/180)*exp(-linspace(0,10,numel(openLoopHeadingSetpoint_rad.data)));
openLoopHeadingSetpoint_rad.data = ...
    openLoopHeadingSetpoint_rad.data + perturbation;

% Set the simulation duration
simulationDuration_s = openLoopHeadingSetpoint_rad.Time(end);

% Set the initial conditions on deltaX (state vector)
initialXPosition_m       = 0;
initialYPosition_m       = 0;
initialSpeed_mPs         = 0;
initialHeading_deg       = 0;
initialPathPosition_none = 0;

fprintf('\nRunning linear Simulink model.')
sim('SimplifiedModel_cm')
parseLogsout;
tscLnSimulink = tsc;
clearvars tsc



%% Now build the lifted system representation
fprintf('\nBuilding lifted system representation.\n')

[F,G] = buildLiftedSytemMatrix(linPlntDisc);

% Difference in initial conditions
deltax0 = zeros([size(linPlntDisc.A.data(:,:,1),2),1]);

% Find difference in control input, resample pert
u = resample(openLoopHeadingSetpoint_rad,linPlntDisc.A.Time);
deltau = squeeze(u.data) - squeeze(linPlntDisc.ctrlInput.data);

deltax = F*deltax0 + G*deltau;

y = deltax + linPlntDisc.stateVector.data(:);

y = reshape(y,size(linPlntDisc.stateVector.data));


%% Plot a bunch of things
if plotSwitch
    figure
    path = pathPosition(linspace(0,1,1000),pathWidth_m,pathHeight_m);
    set(gca,'NextPlot','add');
    
    plot(path(:,1),path(:,2),...
        'DisplayName','Nominal Path','LineWidth',2)
    
    plot(squeeze(tscOrig.xPosition_m.data),squeeze(tscOrig.yPosition_m.data),...
        'DisplayName','Pure Pursuit, Nonlinear Plant, Unperturbed','LineWidth',2)
    

    plot(squeeze(tscLnSimulink.xPosition_m.data),squeeze(tscLnSimulink.yPosition_m.data),...
        'DisplayName','Linear Simulink Model - Perturbed Ctrl','LineWidth',2,'LineStyle',':')
    
    plot(squeeze(y(1,:,:)),squeeze(y(2,:,:)),...
        'DisplayName','Lifted System - Perturned Ctrl')
    
    legend
    set(gca,'FontSize',24)

end


