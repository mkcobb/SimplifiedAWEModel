close all

s = linspace(0,1,1000);
p = pathPositionWithWarping(s,100,10,-1);

 plot(p(:,1),p(:,2))

