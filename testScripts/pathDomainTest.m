% 1) Run the nonlinear time domain plant with the time-domain path 
% following controller.  This produces the path-paramterized trajectories
% to linearize around.
% 2) Run the linear, time-domain plant subject to the path domain 
% controller implementing the control trajectory produced by step 1,
% the results should match exactly
% 3) Alter the control trajectory slightly
% 4) Run the nonlinear, time-domain plant subject to the altered control
% trajetory using the path domain controller
% 5) Run the linear, time-domain plant subject to the altered control
% trajectory using the path domain controller
% Hopefully the results from 5 and 6 are pretty similiar.

close all;clear;clc;
%% Step 1
SimplifiedModel_init
VSSC_CONTROLLER = 1; % Set the controller to the pure pursuit, time domain controller
VSSC_PLANT      = 1; % Set the plant to nonlinear, time domain
SimplifiedModel_prrn

numberOfLaps_none = 1;
simulationDuration_s = inf;

sim('SimplifiedModel_cm')
tsc1 = parseLogsout;
clearvars logsout tsc

%% Step 2
VSSC_CONTROLLER = 4; % Set the controller to the path domain controller
VSSC_PLANT      = 2; % Set the plant to linear, time domain
SimplifiedModel_prrn
pathPositionLookup    = squeeze(tsc1.currentPathPosition_none.data(1:end-1));
headingSetpointLookup = squeeze(tsc1.headingSetpoint_rad.data(1:end-1));
linearizationStateTrajectory   = tsc1.stateVector;
linearizationControlTrajectory = tsc1.headingSetpoint_rad;
sim('SimplifiedModel_cm')
tsc2 = parseLogsout;
clearvars logsout tsc

%% Step 3
headingSetpointLookup = 1.01*squeeze(tsc1.headingSetpoint_rad.data(1:end-1));

%% Step 4
VSSC_CONTROLLER = 4; % Set the controller to the path domain controller
VSSC_PLANT      = 1; % Set the plant to nonlinear, time domain
plantVariants_prrn
sim('SimplifiedModel_cm')
tsc4 = parseLogsout;
clearvars logsout tsc

%% Step 5
VSSC_CONTROLLER = 4; % Set the controller to the pure pursuit, path domain controller
VSSC_PLANT      = 2; % Set the plant to linear, time domain
plantVariants_prrn
sim('SimplifiedModel_cm')
tsc5 = parseLogsout;
clearvars logsout tsc

%% Step 6
VSSC_CONTROLLER = 4; % Set the controller to the pure pursuit, path domain controller
VSSC_PLANT      = 3; % Set the plant to linear, path domain
plantVariants_prrn
headingSetpointLookup     = squeeze(tsc1.headingSetpoint_rad.data(1:end-1));

linearizationPathPosition = squeeze(tsc1.currentPathPosition_none.data(1:end-1));
linearizationStateTrajectory   = tsc1.stateVector.data;
linearizationControlTrajectory = tsc1.headingSetpoint_rad.data;

sim('SimplifiedModel_cm')
tsc6 = parseLogsout;
clearvars logsout tsc


%% Plot things
close all
p = pathPosition(linspace(0,1,1000),pathWidth_m,pathHeight_m);
figure
set(gca,'NextPlot','add')
plot(p(:,1),p(:,2),...
    'DisplayName','Path','LineWidth',5,...
    'LineStyle','--','Color','k')
plot(squeeze(tsc1.xPosition_m.data),...
    squeeze(tsc1.yPosition_m.data),...
    'DisplayName','t-Dom Ctrl, NL t-Dom Plnt',...
    'LineWidth',2,'Color','b');
plot(squeeze(tsc2.xPosition_m.data),...
    squeeze(tsc2.yPosition_m.data),...
    'DisplayName','s-Dom Ctrl, Lin t-Dom Plnt, Unprt Ctrl Sig',...
    'LineWidth',2,'LineStyle',':','Color','b');
plot(squeeze(tsc4.xPosition_m.data),...
    squeeze(tsc4.yPosition_m.data),...
    'DisplayName','s-Dom Ctrl, NL t-Dom Plnt, Pertb Ctrl Sig',...
    'LineWidth',2,'Color','r');
plot(squeeze(tsc5.xPosition_m.data),...
    squeeze(tsc5.yPosition_m.data),...
    'DisplayName','s-Dom Ctrl, Lin t-Dom Plnt, Pertb Ctrl Sig',...
    'LineWidth',2,'LineStyle',':','Color','r');
plot(squeeze(tsc6.xPosition_m.data),...
    squeeze(tsc6.yPosition_m.data),...
    'DisplayName','s-Dom Ctrl, Lin s-Dom Plnt, Unprt Ctrl Sig',...
    'LineWidth',2,'LineStyle',':','Color','r');
grid on
legend();
title('Comparison of Different Plant Behaviors')
set(gca,'FontSize',24)