% Script initializes variants in base workspace
controllerNames = {'purePursuit','openLoop','purePursuitWithWarping'};
VSSC_CONTROLLER = 1;
for ii = 1:length(controllerNames)
eval(['VSS_' upper(controllerNames{ii}) '_CTRL= Simulink.Variant(''VSSC_CONTROLLER==' num2str(ii) ''');'])
end
clearvars controllerNames