function tscCrop = cropTSC2LapNumber(tsc,lapNum)
names = fieldnames(tsc);

% Get data for the specified lap
mask  = tsc.lapNumber.data==lapNum;
startTime = tsc.lapNumber.time(find(mask,1,'first')); % Start time is the first nonzero element
endTime   = tsc.lapNumber.time(find(mask,1,'last' )); % End time is the last nonzero element

for ii = 1:length(names)
   tscCrop.(names{ii}) =  getsampleusingtime(tsc.(names{ii}),startTime,endTime);
end

% Get data that is monotonically increasing


mask = false(size(tscCrop.currentPathPosition_none.data));


% Starting at the middle, march to the end, stop when we hit a decreasing
% point (kind of a flood-fill algorithm)
for ii = round(numel(mask)/2) : numel(mask)-1
   if tscCrop.currentPathPosition_none.data(ii+1)>tscCrop.currentPathPosition_none.data(ii)
      mask(ii) = true; 
   else
       mask(ii) = true; 
       break
   end
end
for ii = round(numel(mask)/2) :-1: 2
   if tscCrop.currentPathPosition_none.data(ii-1)<tscCrop.currentPathPosition_none.data(ii)
      mask(ii) = true; 
   else
       
       break
   end
end
times = tscCrop.currentPathPosition_none.Time(mask);
startTime = times(1);
endTime = times(end);

for ii = 1:length(names)
   tscCrop.(names{ii}) =  getsampleusingtime(tscCrop.(names{ii}),startTime,endTime);
   tscCrop.(names{ii}).Time = tscCrop.(names{ii}).Time - tscCrop.(names{ii}).Time(1);
end


end