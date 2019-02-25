brkpts = brkpts(:);
xVals = xVals(:);
yVals = yVals(:);

brkpts = [brkpts(1:end-1)-(brkpts(end)-brkpts(1));brkpts;brkpts(2:end)+(brkpts(end)-brkpts(1))];

xVals = [xVals(1:end-1)-(xVals(end)-xVals(1));xVals;xVals(2:end)+(xVals(end)-xVals(1))];
yVals = [yVals(1:end-1)-(yVals(end)-yVals(1));yVals;yVals(2:end)+(yVals(end)-yVals(1))];
% plot(brkpts)
% plot(xVals,yVals)
