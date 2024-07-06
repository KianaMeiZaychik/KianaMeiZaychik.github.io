function Xk = DFTmat(xn)
%UNTITLED 此处提供此函数的摘要
%   此处提供详细说明
N=length(xn);
n=0:N-1;k=n;nk=n'*k;
WN=exp(-j*2*pi/N);
Wnk=WN.^nk;
Xk=xn*Wnk;
end