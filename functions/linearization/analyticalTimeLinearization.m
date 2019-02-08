% Get a bunch of constants from the base workspace
headingTau  = evalin('base','headingTimeConstant_s');
speedTau    = evalin('base','speedTimeConstant_s');
vss         = evalin('base','maxSteadySpeed_mPs');
w           = evalin('base','pathWidth_m');
h           = evalin('base','pathHeight_m');

phi         = parameterizationVariable;

% vet vectors (parameterized over path variable)
x = squeeze(linPlnt.stateVector.data(1,:,:));
y = squeeze(linPlnt.stateVector.data(2,:,:));
v = squeeze(linPlnt.stateVector.data(3,:,:));
psi = squeeze(linPlnt.stateVector.data(4,:,:));
psisp = squeeze(linPlnt.ctrlInput.data);



linPlnt.A.data(1:4,1:2,:) = 0;
linPlnt.A.data(4,3,:) = 0;

linPlnt.A.data(1,3,:) = cos(psi);
linPlnt.A.data(2,3,:) = sin(psi);

linPlnt.A.data(3,3,:) = -1/speedTau;

linPlnt.A.data(1,4,:) = -v.*sin(psi);

linPlnt.A.data(2,4,:) = v.*cos(psi);

linPlnt.A.data(3,4,:) = -2.*cos(psi).*sin(psi).*vss/speedTau;

linPlnt.A.data(4,4,:) = -1/headingTau;


linPlnt.B.data(1:3,1,:) = 0;

linPlnt.B.data(4,1,:) = 1/headingTau;
