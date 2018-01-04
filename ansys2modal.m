function [dzdt]=ansys2modal(t,z,A,B,F,tF)
F=interp1(tF,F',t);
F=F';
dzdt=A*z+B*F;
% w=C*z+D*F;


