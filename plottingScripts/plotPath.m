clear;close all;clc

SimplifiedModel_init

p = pathPosition(linspace(0,1,1000),pathWidth_m,pathHeight_m);

plot(p(:,1),p(:,2),'LineWidth',2)
box off
set(gca,'FontSize',32)
xlabel('$p_1$')
ylabel('$p_2$')
