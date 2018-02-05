function[ywA,ywD] = wdriftcorrect(yw,wname,N,ord)

[C,L] = wavedec(yw,ord,wname);
Ctmp = C;
C(sum(L(1:N)):end) = 0;
Ctmp(1:sum(L(1:N))) = 0; 

% Reconstruction after detail coefficient removal
ywA = waverec(C,L,wname);

% Reconstruction after approximation coefficient removal - drift corrected signal
ywD = waverec(Ctmp,L,wname); 

end