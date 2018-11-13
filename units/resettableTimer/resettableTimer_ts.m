close all
clear
clc
sim('resettableTimer_th')

subplot(2,1,1)
time_s.plot

subplot(2,1,2)
pulse.plot

linkaxes(findall(gca,'Type','axes'),'x')

