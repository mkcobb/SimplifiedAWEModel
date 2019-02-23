function [F,G] = buildLiftedSytemMatrix(linPlantDisc)
A = linPlantDisc.A.Data;
B = linPlantDisc.B.Data;

% Preallocate F
F = cell(size(A,3),1);
F{1} = A(:,:,1);

% Preallocate G
G = cell(size(B,3),size(B,3));
G(:,:) = {zeros(size(B(:,:,1)))};
G{1,1} = B(:,:,1);

for ii = 2:size(B,3)
    F{ii} = A(:,:,ii)*F{ii-1};
    G{ii,ii} = B(:,:,ii); % Set the diagonal element
    
    for jj = 1:ii-1 % set lower half elements by multiplying with the line above
       G{ii,jj} = A(:,:,ii)*G{ii-1,jj};
    end
    
end
F = cell2mat(F);
G = cell2mat(G);
end

