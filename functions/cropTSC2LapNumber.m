function tscCrop = cropTSC2LapNumber(tsc,lapNum)

mask = and(tsc.lapNumber.data==lapNum,[1;diff(tsc.currentPathPosition_none.data)>0]);
times = tsc.lapNumber.time(mask);
times = [times(1) times(end-1)];
names = fieldnames(tsc);

for ii = 1:length(names)
   tscCrop.(names{ii}) =  getsampleusingtime(tsc.(names{ii}),times(1),times(2));
end


end