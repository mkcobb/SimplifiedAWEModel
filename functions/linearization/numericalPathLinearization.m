% Script to do the numerical path linearization
fprintf(['\nWARNING: The numerical path linearization is actually calculating',...
    '\nthe linearization of dx/ds where s is the path arc length',...
    '\nNOT the path variable itself. We need to fix this.\n'])
for ii = 1:size(linPlnt.ctrlInput.data,3)
    % Set the initial condition on the path projection
    initialPathPosition_none =  parameterizationVariable(ii);
    % Run linmod
    x = linmod('linearizationPlant_cm',...
        linPlnt.stateVector.data(:,:,ii),...
        linPlnt.ctrlInput.data(ii));
    % Only keep the ones that don't include the discrete state (unit delay)
    linPlnt.A.data(:,:,ii) = x.a;
    linPlnt.B.data(:,:,ii) = x.b;
end