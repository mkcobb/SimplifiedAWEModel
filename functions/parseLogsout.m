function parseLogsout()

logsout = evalin('base','logsout');

names = logsout.getElementNames;

% get rid of unnamed signals
names = names(cellfun(@(x) ~isempty(x),names));

for ii = 1:length(names)
    ts = logsout.getElement(names{ii});
    tsc.(names{ii}) = ts.Values;
    
end

assignin('base','tsc',tsc);
end