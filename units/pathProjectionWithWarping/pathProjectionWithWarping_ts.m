close all
clear

pathProjectionWithWarping_init

pathWidth_m = 100;
pathHeight_m = 10;
xPos = 49;
yPos = -1;

warpingFactor_none =-0.8;

sim('pathProjectionWithWarping_th')

p = pathPositionWithWarping(linspace(0,1,1000),pathWidth_m,pathHeight_m,warpingFactor_none);

pos = pathPositionWithWarping(simout.data(end),pathWidth_m,pathHeight_m,warpingFactor_none);

plot(p(:,1),p(:,2));
grid on
hold on
scatter(xPos,yPos)
scatter(pos(1),pos(2))
axis equal