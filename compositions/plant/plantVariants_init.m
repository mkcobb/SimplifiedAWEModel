plantNames = {'nonlinear','linear'};

VSSC_PLANT = 1;

for ii = 1:length(plantNames)
eval(['VSS_' upper(plantNames{ii}) '_PLNT= Simulink.Variant(''VSSC_PLANT==' num2str(ii) ''');'])
end

clearvars plantNames