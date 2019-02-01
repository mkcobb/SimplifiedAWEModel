close all;clear;clc

SimplifiedModel_init
VSSC_CONTROLLER = 3;
SimplifiedModel_prrn

simulationDuration_s = inf;
headingTimeConstant_s = 0.02;
warpingFactors = -1.6:0.2:0.8;
numberOfLaps_none = 3;
offsetLeadDistance_none = 5e-3;
pathWidth_m = 100;
pathHeight_m = 10;

figure;grid on;set(gca,'NextPlot','add','FontSize',16)
title('Comparison of Paths')
legend

for ii = 1:length(warpingFactors)
    warpingFactor_none = warpingFactors(ii);
    sim('SimplifiedModel_cm')
    parseLogsout
    p = pathPositionWithWarping(linspace(0,1,1000),pathWidth_m,pathHeight_m,warpingFactor_none);
    
    plot(p(:,1),p(:,2),'DisplayName',['$w_F$ = ' num2str(warpingFactor_none,2) ' Nominal Path'])
    
    plot(squeeze(tsc.xPosition_m.data(1,1,:)),...
        squeeze(tsc.yPosition_m.data(1,1,:)),...
        'LineStyle','--',...
        'DisplayName',['$w_F$ = ' num2str(warpingFactor_none,2) ' Achieved Path'])
    
    performances(ii) = tsc.lapAverageSpeed_mPs.data(end-1);
    times(ii) = tsc.lapTime_s.data(end-1);
end

figure
subplot(1,2,1)
plot(warpingFactors,performances.^3)
xlabel('Warping Factor')
ylabel('Lap Average Speed$^3$ [m/s]')
set(gca,'FontSize',16)

subplot(1,2,2)
plot(warpingFactors,times)
xlabel('Warping Factor')
ylabel('Lap Time [s]')
set(gca,'FontSize',16)

