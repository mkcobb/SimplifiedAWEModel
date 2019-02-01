function h = plotLinearPlant(linPlnt)
% This plots the elements of the A and B matrices of the path-linearized
% plant as functions of the path variable

h.fig.A = figure;
axNum = 1;
for ii = 1:size(linPlnt.A.data(:,:,1),1)
    for jj = 1:size(linPlnt.A.data(:,:,1),2)
        subplot(size(linPlnt.A.data(:,:,1),1),size(linPlnt.A.data(:,:,1),2),axNum)
        plot(linPlnt.A.Time,...
            squeeze(linPlnt.A.data(ii,jj,:)))
        hold on
        
%         plot(ALinSim.time,squeeze(ALinSim.data(ii,jj,:)))
        
        ylabel(['A( ' num2str(ii) ' , ' num2str(jj) ' )'])
        grid on
        axNum = axNum+1;
    end
end
linkaxes(findall(gcf,'Type','axes'),'x')
set(findall(gcf,'Type','axes'),'FontSize',16)


h.fig.B = figure;
axNum = 1;
for ii = 1:size(linPlnt.B.data(:,:,1),1)
    for jj = 1:size(linPlnt.B.data(:,:,1),2)
        subplot(size(linPlnt.B.data(:,:,1),1),size(linPlnt.B.data(:,:,1),2),axNum)
        plot(linPlnt.B.Time,...
            squeeze(linPlnt.B.data(ii,jj,:)))
        hold on
        try
            plot(BLinSim.time,squeeze(BLinSim.data(ii,jj,:)))
        catch
        end
        ylabel(['B( ' num2str(ii) ' , ' num2str(jj) ' )'])
        grid on
        axNum = axNum+1;
    end
end


end