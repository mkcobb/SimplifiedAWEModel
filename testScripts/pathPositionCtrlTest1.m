% Script to test the path-position based controller:
% Run the normal path-following controller to get the heading setpoint as a
% function of the path position.  Then run the path-position based
% controller to see if this gives the same results.
% Set some things up
close all
clear
SimplifiedModel_init
VSSC_CONTROLLER = 1;
VSSC_PLANT = 1;
SimplifiedModel_prrn;

simulationDuration_s = inf;
numberOfLaps_none = 1;
offsetLeadDistance_none = 0.01;

p = pathPosition(linspace(0,1,1000),pathWidth_m,pathHeight_m);

%% Run the normal path following controller
sim('SimplifiedModel_cm')
parseLogsout
tscPathFollowing = tsc;
clearvars tsc

%% Run the position-based controller
VSSC_CONTROLLER = 4;
SimplifiedModel_prrn;
% tscPathFollowingClean = pathBasedLookupCleanup(tscPathFollowing,'mean');
pathPositionLookup = squeeze(tscPathFollowing.currentPathPosition_none.data(1:end-1));
headingSetpointLookup = squeeze(tscPathFollowing.headingSetpoint_rad.data(1:end-1));
sim('SimplifiedModel_cm')
parseLogsout
tscPathPosition = tsc;
clearvars tsc;

%% Plot things
figure
plot(p(:,1),p(:,2),'DisplayName','Path')
hold on
plot(squeeze(tscPathFollowing.xPosition_m.data),...
    squeeze(tscPathFollowing.yPosition_m.data),...
    'DisplayName','Path Following Ctrl')
plot(squeeze(tscPathPosition.xPosition_m.data),...
    squeeze(tscPathPosition.yPosition_m.data),...
    'DisplayName','Path Position Based Ctrl')
legend
set(gca,'FontSize',24)
legend

figure
plot(squeeze(tscPathFollowing.currentPathPosition_none.data),...
    squeeze(tscPathFollowing.headingSetpoint_rad.data),...
    'DisplayName','Path Following Ctrl')
hold on
plot(squeeze(tscPathPosition.currentPathPosition_none.data),...
    squeeze(tscPathPosition.headingSetpoint_rad.data),...
    'DisplayName','Path Position Based Ctrl')
xlabel('Path Position')
ylabel('Control Input, Heading Setpoint [rad]')
set(gca,'FontSize',24)
legend