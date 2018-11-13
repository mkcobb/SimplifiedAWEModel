close all

s = linspace(0,1,1000);
p = pathPositionWithWarping(s,100,10,0.2);


plot(p(:,1),p(:,2))

