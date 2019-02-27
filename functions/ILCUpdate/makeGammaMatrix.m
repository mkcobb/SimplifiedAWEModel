function [gammaMat] = makeGammaMatrix(Ad,Bd,ACont, BCont, pathStepTimes,deltauNom)
% Generate the gamma matrix to capture the effect of changes in pathstep
% timing
Tnj = diff(pathStepTimes);
numStates = size(Ad,1);
N = numStates*length(Tnj);

betaMat = zeros(N);
for m=1:numStates:N
    currPathStep = ceil(m/numStates);
    for n=1:currPathStep
        cumulativeAd = eye(size(Ad,1));
        for i=n:currPathStep-1
           cumulativeAd = cumulativeAd*Ad(:,:,i);
        end
        
        if currPathStep==n
            betaMat(m:m+numStates-1,n) = Ad(:,:,n)*BCont(:,:,n);
        else
            %Need to check if ACont(:,:,n) is the right variable to be
            %using here
            betaMat(m:m+numStates-1,n) = (currPathStep-n)*ACont(:,:,n)*cumulativeAd*Bd(:,:,n)+cumulativeAd*Ad(:,:,n)*BCont(:,:,n);
        end
    end
end
%Check if this is necessary. Need to know what the size of deltauNom is
deltauNomExpand = repelem(deltauNom,numStates);
gammaMat = betaMat*diag(deltauNomExpand);
end

