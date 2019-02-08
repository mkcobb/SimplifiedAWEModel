% Script to do the path linearization with respect to the path variable

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

% Calculate some intermediate variables
c = w.^2.* cos(4.* pi.* phi) + 4.* h.^2.* cos(8.* pi.* phi) +  2.* w.* sin(2.* pi.* phi).* x - 8.* h.* sin(4.* pi.* phi).* y;

d1 =  w.* cos(2.* pi.* phi).* cos(psi) -  2.* h.* cos(4.* pi.* phi).* sin(psi);

d2 = -d1;

a1 = (cos(psi).^2 .*vss - v);

% Calculate elements of A
num11 = 2.*pi.*w.*cos(psi).*sin(2.*pi.*phi);
den11 = d1;
linPlnt.A.data(1,1,:) = num11./den11;

num21 = 2.*pi.*w.*sin(psi).*sin(2.*pi.*phi);
den21 = d1;
linPlnt.A.data(2,1,:) = num21./den21;

num31 = 2.*pi.*w.*sin(2.*pi.*phi).*a1;
den31 = d1.*speedTau.*v;
linPlnt.A.data(3,1,:) = num31./den31;

num41 = 2.*pi.*w.*sin(2.*pi.*phi).*(psisp-psi);
den41 = d1.*headingTau.*v;
linPlnt.A.data(4,1,:) = num41./den41;

num12 = 8.*pi.*h.*cos(psi).*sin(4.*pi.*phi);
den12 = d2;
linPlnt.A.data(1,2,:) = num12./den12;

num22 = 8.*pi.*h.*sin(psi).*sin(4.*pi.*phi);
den22 = d2;
linPlnt.A.data(2,2,:) = num22./den22;

num32 = -8.*h.*pi.*sin(4.*pi.*phi).*a1;
den32 = d1.*speedTau.*v;
linPlnt.A.data(3,2,:) = num32./den32;

num42 = 8.*h.*pi.*sin(4.*pi.*phi).*(psi-psisp);
den42 = d1.*headingTau.*v;
linPlnt.A.data(4,2,:) = num42./den42;

linPlnt.A.data(1,3,:) = 0;
linPlnt.A.data(2,3,:) = 0;

num33 = -c.*pi.*cos(psi).^2.*vss;
den33 = d1.*speedTau.*v.^2;
linPlnt.A.data(3,3,:) = num33./den33;

num43 = c.*pi.*(psi-psisp);
den43 = d1.*headingTau.*v.^2;
linPlnt.A.data(4,3,:) = num43./den43;

num14 = 2.*c.*h.*pi.*cos(4.*pi.*phi);
den14 = d1.^2;
linPlnt.A.data(1,4,:) = num14./den14;

num24 = c.*pi.*w.*cos(2.*pi.*phi);
den24 = d1.^2;
linPlnt.A.data(2,4,:) = num24./den24;

num34 = c.* pi.* (cos(psi).* (h.* cos(4.* pi.* phi).* (-3 + cos(2.* psi)) +  w.* cos(2.* pi.* phi).* cos(psi).* sin(psi)).* vss + (2.* h.* cos(4.* pi.* phi).* cos(psi) +  w.* cos(2.* pi.* phi).* sin(psi)).* v);
den34 = d1.^2.*speedTau.*v;
linPlnt.A.data(3,4,:) = -num34./den34;

% num44 = c.* pi.* (w.* cos(2.* pi.* phi).* cos(psi) -    2.* h.* cos(4.* pi.* phi).* sin(psi) + (2.* h.* cos(4.* pi.* phi).* cos(psi) +       w.* cos(2.* pi.* phi).* sin(psi)).* psi - (2.* h.* cos(4.* pi.* phi).* cos(psi) + w.* cos(2.* pi.* phi).* sin(psi)).*psisp);
num44 = c.* pi.* (d1 + (2.* h.* cos(4.* pi.* phi).* cos(psi) +...
    w.* cos(2.* pi.* phi).* sin(psi)).* psi -...
    (2.* h.* cos(4.* pi.* phi).* cos(psi) +...
    w.* cos(2.* pi.* phi).* sin(psi)).* psisp);
den44 = d1.^2.*headingTau.*v;
linPlnt.A.data(4,4,:) = -num44./den44;

% Calculate elements of B
linPlnt.B.data(1,1,:) = 0;
linPlnt.B.data(2,1,:) = 0;
linPlnt.B.data(3,1,:) = 0;
linPlnt.B.data(4,1,:) = (c.*pi)./(d1.*headingTau.*v);

