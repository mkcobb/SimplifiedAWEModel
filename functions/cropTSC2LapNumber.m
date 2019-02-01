function tscCrop = cropTSC2LapNumber(tsc,lapNum)

times = tsc.lapNumber.time(tsc.lapNumber.data==lapNum);
times = [times(1) times(end)];
names = fieldnames(tsc);

for ii = 1:length(names)
   tscCrop.(names{ii}) =  getsampleusingtime(tsc.(names{ii}),times(1),times(2));
    
end


end