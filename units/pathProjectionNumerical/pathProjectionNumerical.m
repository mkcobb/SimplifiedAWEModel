function [s,lapTicker] = pathProjectionNumerical(s0,x,y,brkpts, xVals, yVals, sDist)

fHandle = @(s) sqrt(sum(([x y]-[interp1(brkpts,xVals,s) interp1(brkpts,yVals,s)]).^2,2));

s = goldenSection(fHandle,s0-sDist,s0+sDist,1e-5);

% if s-s0<1e-5
%     s = s0;
% end

lapTicker = 0;
% Make negative values positive (eg -0.1 should be 0.9)

% Round to the nearest 5 decimal places
% This step is actually critical because it ensures that some path
% positions are read as exactly 0 and 1, thus making
% resampling/interpolating possible in some scripts.

s = round(s,6);
while s<0
    s = s+1;
end

if s>1
    lapTicker = 1;
end


% Wrap values greater than 1 into the 0,1 range (eg 1.1 should be 0.1)
s = rem(s,1);

end
