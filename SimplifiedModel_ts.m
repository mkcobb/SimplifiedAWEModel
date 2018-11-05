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
plot(tsc.xPosition_m.data,tsc.yPosition_m.data,'DisplayName','Position')
plot(tsc.targetX_m.data,tsc.targetY_m.data,'DisplayName','TargetPoint')
plot(tsc.currentPathPositionX_m.data,tsc.currentPathPositionY_m.data,'DisplayName','TargetPoint')

figure
tsc.headingSetpoint_rad.plot;
grid on
hold on
tsc.heading_rad.plot;

