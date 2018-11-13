function r0 = pathPositionWithWarping(s,w,h,wFactor)
% Function to calculate the shape of the path, given the basis paramters, w
% and h.  The input s is the position along the path between 0 (start of
% path) and 1 (end of path).  wFactor is a warping factor between 0 (no
% warping) and 1 (most extreme warping).
% For documentation on how this warping method is performed, see
% warpingMethodDocumentation.jpg
x = (w)*cos((2*s+3/2)*pi);
y = h*cos((2*s+3/2)*pi).*sin((2*s+3/2)*pi);
r0 =[x' y'];
sToReplace = or(and(s>0.125,s<0.125*3),and(s>0.125*5,s<0.125*7));
if sum(sToReplace) || wFactor ~=0
    xStar = (w)*cos((2*0.125+3/2)*pi);
    deltaX = (w)-xStar;
    r2 = zeros(size(r0));
    r2(s<0.5,1) = xStar;
    r2(s>=0.5,1) = -xStar;
    r1 = r0-r2;
    u1 = r1./(sqrt(sum(r1.^2,2)));
    aStar = min([(h/2)*((sqrt(sum(r1.^2,2)))./(abs(r1(:,2)))) ...
        deltaX*((sqrt(sum(r1.^2,2)))./(abs(r1(:,1))))],[],2);
    r6 = r0+wFactor*(r2+aStar.*u1-r0);
    r0(sToReplace,:) = r6(sToReplace,:);
end
end