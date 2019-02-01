function xStar = goldenSection(fHandle,xl,xr,convTol)
% One-dimensional function minimization
tau = 1 - 0.38197;
X = [xl tau*xl+(1-tau)*xr (1-tau)*xl+tau*xr xr];
F = fHandle(X);
for ii = 1:1000
    if F(2)<F(3)
        X(4) = X(3);
        F(4) = F(3);
        X(3) = X(2);
        F(3) = F(2);
        X(2) = tau*X(1)+(1-tau)*X(4);
        F(2) = fHandle(X(2));
    elseif F(3)<F(2)
        X(1) = X(2);
        F(1) = F(2);
        X(2) = X(3);
        F(2) = F(3);
        X(3) = (1-tau)*X(1)+tau*X(4);
        F(3) = fHandle(X(3));
    elseif F(2) == F(3)
        X(1) = X(2);
        X(4) = X(3);
        X(2) = tau*X(1)+(1-tau)*X(4);
        X(3) = (1-tau)*X(1)+tau*X(4);
        F(2) = fHandle(X(2));
        F(3) = fHandle(X(3));
    end
    if abs(X(4)-X(1))<convTol
        break
    end
end
[~,idx] = min(F);
xStar = X(idx);
end