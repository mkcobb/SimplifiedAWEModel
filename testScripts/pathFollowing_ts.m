% Script to test the path following, making sure that works
% Should produce x-y plot of position showing that it followed the figure
% 8 and a heading plot showing that the heading tracked the heading
% setpoint asymptotically

close all
clc
SimplifiedModel_init

simulationDuration_s = 20;

sim('SimplifiedModel_cm')

parseLogsout

p = pathPosition(linspace(0,1,1000),pathWidth_m,pathHeight_m);

figure
grid on
plot(p(:,1),p(:,2),'DisplayName','Path')
hold on
plot(squeeze(tsc.xPosition_m.data(1,1,:)),...
    squeeze(tsc.yPosition_m.data(1,1,:)),'DisplayName','Position')
plot(squeeze(tsc.targetX_m.data(1,1,:)),...
    squeeze(tsc.targetY_m.data(1,1,:)),'DisplayName','TargetPoint')
plot(squeeze(tsc.currentPathPositionX_m.data(1,1,:)),...
    squeeze(tsc.currentPathPositionY_m.data(1,1,:)),'DisplayName','TargetPoint')

figure
tsc.headingSetpoint_rad.plot;
grid on
hold on
tsc.heading_rad.plot;

