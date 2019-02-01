function tanVec = calculatePathTangentVector(...
    pathVariable,pathWidth,pathHeight)
% For an explanation of how to derive the tangent vector, see
% SFFrame_v2.nb
% In the documentation folder
phi = pathVariable;
w = pathWidth;
h = pathHeight;

tanVec = [w*cos(2*pi*phi) -2*h*cos(4*pi*phi)];
tanVec = tanVec./sqrt(w^2*cos(2*pi*phi)^2+4*h^2*cos(4*pi*phi)^2);
end