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
mask = diff(tscCrop.currentPathPosition_none.data)>0;
endIndex   = find(mask,1,'last' );
if endIndex == length(mask)
   mask = [mask ; 1];
else
    mask = [mask ; 0];
end
endIndex   = find(mask,1,'last' );
startIndex = find(mask,1,'first');

startTime = tscCrop.lapNumber.time(startIndex); % Start time is the first nonzero element
endTime   = tscCrop.lapNumber.time(endIndex); % End time is the last nonzero element


for ii = 1:length(names)
   tscCrop.(names{ii}) =  getsampleusingtime(tscCrop.(names{ii}),startTime,endTime);
   tscCrop.(names{ii}).Time = tscCrop.(names{ii}).Time - tscCrop.(names{ii}).Time(1);
end


end