close all;clear

previousPathPosition = 0.99;

pathProjection_init;
pathPosition_init;
positionX_m = 20;
positionY_m = -2;


sim('pathProjection_th')

s = linspace(0,1,1000);
path = pathPosition(s,pathWidth_m,pathHeight_m);

closest = pathPosition(out.data(end),pathWidth_m,pathHeight_m);

previous = pathPosition(previousPathPosition,pathWidth_m,pathHeight_m);


figure
plot(path(:,1),path(:,2),'DisplayName','Path')
hold on
grid on
scatter(closest(:,1),closest(:,2),'DisplayName','Closest Point')
scatter(previous(:,1),previous(:,2),'DisplayName','Previous Closest Point')
scatter(positionX_m,positionY_m,'DisplayName','Current Position')

axis square
axis equal
legend

