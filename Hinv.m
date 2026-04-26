function [ H2 ] = Hinv( H )
%Finds the inverse of a Homogeneous transformation
H=squeeze(H);
R=H(1:3,1:3);
O=H(1:3,4);
H2=[R',-R'*O;0,0,0,1];
end

