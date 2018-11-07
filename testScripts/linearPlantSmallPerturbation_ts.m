%% Script to test the linear plant.
% The linearization is done around one trajectory, and the simulation with
% the linear plant uses a slightly perturbed version of that control
% trajectory.  The results should show that for a minimally perturbed
% control signal, we get pretty similiar results.
clear all
close all
clc
% First run the nonlinear plant with the path following controller
SimplifiedModel_init
VSSC_PLANT = 1;
VSSC_CONTROLLER = 1;
SimplifiedModel_prrn

simulationDuration_s = 20;

sim('SimplifiedModel_cm')

parseLogsout

tscNonlinear = tsc;
clearvars tsc logsout

%% Next, run the linear plant with the open loop controller
VSSC_PLANT = 2;
VSSC_CONTROLLER = 2;
SimplifiedModel_prrn

openLoopHeadingSetpoint_rad = tscNonlinear.headingSetpoint_rad;
linearizationStateTrajectoryVector = tscNonlinear.stateVector;
linearizationControlTrajectoryVector = tscNonlinear.headingSetpoint_rad;

% Key point here: perturb the control signal a little bit, we should still
% get a figure 8
openLoopHeadingSetpoint_rad.data = 1.001*openLoopHeadingSetpoint_rad.data;

sim('SimplifiedModel_cm')

parseLogsout

tscLinear = tsc;
clearvars tsc;

%% Plot the results for side-by-side comparison

p = pathPosition(linspace(0,1,1000),pathWidth_m,pathHeight_m);
% Plot the path
figure
grid on
plot(p(:,1),p(:,2),'DisplayName','Path')
hold on
plot(squeeze(tscNonlinear.xPosition_m.data(1,1,:)),...
    squeeze(tscNonlinear.yPosition_m.data(1,1,:)),'DisplayName','Position - Nonlinear')
plot(squeeze(tscLinear.xPosition_m.data(1,1,:)),...
    squeeze(tscLinear.yPosition_m.data(1,1,:)),'DisplayName','Position - Linear')
xlabel('x Position, [m]')
ylabel('y Position, [m]')
title('Comparison of Positions')
legend
set(gca,'FontSize',24)


figure
tscNonlinear.headingSetpoint_rad.plot;
grid on
hold on
tscLinear.headingSetpoint_rad.plot;
xlabel('Time, t [s]')
ylabel('Heading Setpoint')
title('Comparison of Control Signals')
legend
set(gca,'FontSize',24)

figure
tscNonlinear.speed_mPs.plot;
grid on
hold on
tscLinear.speed_mPs.plot;
xlabel('Time, t [s]')
ylabel('Speed, v [mPs]')
title('Comparison of System Speed')
legend
set(gca,'FontSize',24)
