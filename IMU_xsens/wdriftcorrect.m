function[ywA,ywD] = wdriftcorrect(yw,wname,N,ord)
% Obtains low frequency approximation signal (ywA) and detail signal (ywD)
% after wavelet decomposition

% yw: intput signal
% wname: wavelet type name
% N: level of detail to be conserved
% ord: decomposition level to be attained 

% Wavelet deconstruction of signal into approximation and detail
% coefficients
[C,L] = wavedec(yw,ord,wname);
Ctmp = C;
% Detail coefficients are removed
C(sum(L(1:N)):end) = 0;
% Approximation and part of detail coefficients are removed
Ctmp(1:sum(L(1:N))) = 0; 

% Signal reconstruction after detail coefficient removal
ywA = waverec(C,L,wname);

% Reconstruction after approximation coefficient removal - drift corrected signal
ywD = waverec(Ctmp,L,wname); 

end