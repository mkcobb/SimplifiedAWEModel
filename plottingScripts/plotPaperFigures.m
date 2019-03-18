%%
close all;
numIterationsToPlot = 50;
fontSize = 48;
figureDir = 'C:\Users\mitch\GoogleDrive_School\Mitchell-Research\SimplifiedAWEModel_UoM\output\figures';
exampleLaps = [1 11 21];
windSpeed = 4;

%% Plot Velocity Polar
figure
ylabel('Max Steady State Speed [m/s]')
title('Velocity Polar')
angles = linspace(-2*pi,2*pi,1000);
radii = maxSteadySpeed_mPs*cos(angles).^2;
polarplot(angles,radii,'LineWidth',2,'Color',[0 0 0]);
set(gca,'FontSize',fontSize)
% set(gca,'ThetaLim',[0 90])
savePlot(gcf,figureDir,'VelocityPolar')

%% Plot Performance Index vs Iteration
figure
set(gca,'NextPlot','add')
set(gca,'FontSize',fontSize)
xlabel('Iteration Number')
ylabel('Mean Speed [m/s]')
title('Performance Index Vs Iteration')
for ii = 1:numIterationsToPlot
    scatter(ii, mean(tscc{ii}.speed_mPs.data),...
        'CData',[0 0 0 ],'MarkerFaceColor','Flat','SizeData',60)
end
savePlot(gcf,figureDir,'PerformanceIndexVsIterationNumber')

%% Plot EAR
figure
set(gca,'NextPlot','add')
set(gca,'FontSize',fontSize)
xlabel('Iteration Number')
ylabel('EAR')
title('Energy Augmentation Ratio')
for ii = 1:numIterationsToPlot
    time         = tscc{ii}.currentPathPosition_none.Time - tscc{ii}.currentPathPosition_none.Time(1);
    speed        = squeeze(tscc{ii}.speed_mPs.data);
    heading      = squeeze(tscc{ii}.heading_rad.data);
    velocity     = [speed.*cos(heading) speed.*sin(heading)];
    windVelocity = windSpeed*[zeros(size(velocity,1),1) -ones(size(velocity,1),1)];
    apparentWind = windVelocity-velocity;
    powerCrosswind        = sqrt(sum(apparentWind.^2,2)).^3;
    powerStandstill       = sqrt(sum((windSpeed*ones(size(powerCrosswind))).^2,2)).^3;
    meanPowerCrosswind    = trapz(time,powerCrosswind);
    meanPowerStandstill   = trapz(time,powerStandstill);
    scatter(ii, meanPowerCrosswind/meanPowerStandstill,...
        'CData',[0 0 0 ],'MarkerFaceColor','Flat','SizeData',60)
end
savePlot(gcf,figureDir,'EAR')

%% Plot Instantaneous EAR
figure
set(gca,'NextPlot','add')
set(gca,'FontSize',fontSize)
xlabel('Iteration Number')
ylabel('Instant. EAR')
title('Instantaneous Energy Augmentation Ratio')
for ii = 1:length(exampleLaps)
    time         = tscc{exampleLaps(ii)}.currentPathPosition_none.Time - tscc{exampleLaps(ii)}.currentPathPosition_none.Time(1);
    speed        = squeeze(tscc{exampleLaps(ii)}.speed_mPs.data);
    heading      = squeeze(tscc{exampleLaps(ii)}.heading_rad.data);
    velocity     = [speed.*cos(heading) speed.*sin(heading)];
    windVelocity = windSpeed*[zeros(size(velocity,1),1) -ones(size(velocity,1),1)];
    apparentWind = windVelocity-velocity;
    powerCrosswind        = sqrt(sum(apparentWind.^2,2)).^3;
    powerStandstill       = sqrt(sum((windSpeed*ones(size(powerCrosswind))).^2,2)).^3;
    plot(time,powerCrosswind./powerStandstill,'LineWidth',2,...
        'DisplayName',sprintf('Iteration %d',exampleLaps(ii)))
end
leg = legend;
leg.Location = 'best';
leg.FontSize = 32;
savePlot(gcf,figureDir,'InstantEAR')

%% Plot Example Paths
figure
set(gca,'NextPlot','add')
set(gca,'FontSize',fontSize)
xlabel('x Position [m]')
ylabel('y Position [m]')
title('Example Paths')
for ii = 1:length(exampleLaps)
    plot(squeeze(tscc{exampleLaps(ii)}.xPosition_m.data),...
         squeeze(tscc{exampleLaps(ii)}.yPosition_m.data),...
        'DisplayName',sprintf('Iteration %d',exampleLaps(ii)),...
        'LineWidth',2)
end
leg = legend;
leg.Location = 'south';
leg.FontSize = 32;
savePlot(gcf,figureDir,'ExampleLaps')
 
%% Plot Instantaneous Power
figure
set(gca,'NextPlot','add')
set(gca,'FontSize',fontSize)
xlabel('Time, t [s]')
ylabel('Instant. Power, [$(m/s)^3$]')
title('Example Instantaneous Power Output')
for ii = 1:length(exampleLaps)
    time         = tscc{exampleLaps(ii)}.currentPathPosition_none.Time;
    speed        = squeeze(tscc{exampleLaps(ii)}.speed_mPs.data);
    heading      = squeeze(tscc{exampleLaps(ii)}.heading_rad.data);
    velocity     = [speed.*cos(heading) speed.*sin(heading)];
    windVelocity = windSpeed*[zeros(size(velocity,1),1) -ones(size(velocity,1),1)];
    apparentWind = windVelocity-velocity;
    powerCrosswind        = (sqrt(sum(apparentWind.^2,2))).^3;
    plot(time,powerCrosswind,'LineWidth',2,...
        'DisplayName',sprintf('Iteration %d',exampleLaps(ii)))
end
leg = legend;
leg.Location = 'best';
leg.FontSize = 32;
savePlot(gcf,figureDir,'InstantaneousPower')

%% Crop all the images in the specified folder
cropImages('C:\Users\mitch\GoogleDrive_School\Mitchell-Research\SimplifiedAWEModel_UoM\output\figures\png')

