function s = pathProjection(s0,x,y,w,h,sDist)
fHandle = @(s) sqrt(sum(([x y]-pathPosition(s,w,h)).^2,2));
s = goldenSection(fHandle,s0,s0+sDist,1e-6);
s = rem(s,1);
end