function s = pathProjection(s0,x,y,w,h,sDist)
fHandle = @(s) sqrt(sum(([x y]-pathPosition(s,w,h)).^2,2));
s = goldenSection(fHandle,s0,s0+sDist,1e-6);
% Make negative values positive (eg -0.1 should be 0.9)
while s<0
    s = s+1;
end
% Wrap values greater than 1 into the 0,1 range (eg 1.1 should be 0.1)
s = rem(s,1);
end