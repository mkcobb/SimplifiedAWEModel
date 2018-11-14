function r0 = pathPositionWithWarping(s,w,h,wFactor)
% Function to calculate the shape of the path, given the basis paramters, w
% and h.  The input s is the position along the path between 0 (start of
% path) and 1 (end of path).  wFactor is a warping factor between 0 (no
% warping) and 1 (most extreme warping).
% For documentation on how this warping method is performed, see
% warpingMethodPositive.jpg and warpingMethodNegative.jpg
x = (w/2)*cos((2*s+3/2)*pi);
y = h*cos((2*s+3/2)*pi).*sin((2*s+3/2)*pi);
r0 =[x' y'];
sToReplace = or(and(s>0.125,s<0.125*3),and(s>0.125*5,s<0.125*7));

if sum(sToReplace)  && wFactor~=0
    xStar = (w/2)*cos((2*0.125+3/2)*pi);
    if wFactor>0 % Positive warping
        deltaX = (w/2)-xStar;
        r2 = zeros(size(r0));
        r2(s<0.5,1) = xStar;
        r2(s>=0.5,1) = -xStar;
        r1 = r0-r2;
        u1 = r1./(sqrt(sum(r1.^2,2)));
        aStar = min([(h/2)*((sqrt(sum(r1.^2,2)))./(abs(r1(:,2)))) ...
            deltaX*((sqrt(sum(r1.^2,2)))./(abs(r1(:,1))))],[],2);
        r6 = r0+wFactor*(r2+aStar.*u1-r0);
        r0(sToReplace,:) = r6(sToReplace,:);
    else % Negative warping
        r2 = nan(size(r0));
        r2(and(s>0.125*1,s<0.125*3),1) =  xStar;
        r2(and(s>0.125*1,s<0.125*3),2) =  0;
        r2(and(s>0.125*5,s<0.125*7),1) = -xStar;
        r2(and(s>0.125*5,s<0.125*7),2) =  0;
        P1 = nan(size(r0));
        P1(and(s>0.125*1,s<0.125*2),:) = repmat([xStar  -h/2],[sum(and(s>0.125*1,s<0.125*2)),1]);
        P1(and(s>0.125*2,s<0.125*3),:) = repmat([xStar   h/2],[sum(and(s>0.125*2,s<0.125*3)),1]);
        P1(and(s>0.125*5,s<0.125*6),:) = repmat([-xStar -h/2],[sum(and(s>0.125*5,s<0.125*6)),1]);
        P1(and(s>0.125*6,s<0.125*7),:) = repmat([-xStar  h/2],[sum(and(s>0.125*6,s<0.125*7)),1]);
        r4 = nan(size(r0));
        r4(and(s>0.125*1,s<0.125*3),1) = w/2;
        r4(and(s>0.125*1,s<0.125*3),2) = 0;
        r4(and(s>0.125*5,s<0.125*7),1) = -w/2;
        r4(and(s>0.125*5,s<0.125*7),2) = 0;
        rP = P1;
        r1 = r0-r2;
        r3 = r4-rP;
        A = r1(:,2)./r1(:,1);
        B = r3(:,2)./r3(:,1);
        xStarVec = P1(:,1);
        xInt = ((A-B).^-1).*(A.*xStarVec-B.*P1(:,1)+P1(:,2));
        yInt = A.*(xInt - xStarVec);
        rInt = [xInt,yInt];
        r6 = rInt-r0;
        r0(sToReplace,:) = r0(sToReplace,:)+abs(wFactor)*r6(sToReplace,:);
    end
    
end
end