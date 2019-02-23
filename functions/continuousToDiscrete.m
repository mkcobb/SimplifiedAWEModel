function linPlntDisc = continuousToDiscrete(linPlnt,stepSize)

linPlntDisc = linPlnt;
identityMatrix = eye(size(linPlnt.A.data(:,:,1)));
zeroMatrix     = zeros(size(linPlnt.B.data(:,:,1)));
for ii = 1:size(linPlnt.A.data,3)
    %     sys = ss(linPlnt.A.data(:,:,ii),linPlnt.B.data(:,:,ii),...
    %         linPlnt.C.data(:,:,ii),linPlnt.D.data(:,:,ii));
    %     sysd = c2d(sys,stepSize);
    
    
    
    %     linPlntDisc.A.data(:,:,ii) = sysd.A;
    %     linPlntDisc.B.data(:,:,ii) = sysd.B;
    %     linPlntDisc.C.data(:,:,ii) = sysd.C;
    %     linPlntDisc.D.data(:,:,ii) = sysd.D;
    % A = linPlnt.A.data(:,:,ii);
    % B = linPlnt.A.data(:,:,ii);
    % C = eye(size(A));
    % D = zeros(size(B));
    % linPlntDisc.A.data(:,:,ii) = exp(A*stepSize);
    % linPlntDisc.B.data(:,:,ii) = (A\(exp(A*stepSize)-eye(size(A))))*B;
    % linPlntDisc.C.data(:,:,ii) = C;
    % linPlntDisc.D.data(:,:,ii) = D;
    
    [Ad, Bd] = c2d(linPlnt.A.data(:,:,ii),linPlnt.B.data(:,:,ii),stepSize);
    linPlntDisc.A.data(:,:,ii) = Ad;
    linPlntDisc.B.data(:,:,ii) = Bd;
    linPlntDisc.C.data(:,:,ii) = identityMatrix;
    linPlntDisc.D.data(:,:,ii) = zeroMatrix;
end


end