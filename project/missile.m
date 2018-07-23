function [ Tval,Xval,Yval,Zval,Uval,Vval,Wval] = missile( X0,Y0,Z0,m0,mf,Thmag0,theta,phi,Tburn)
%missile This function takes input values of one of the missiles and outputs the Tval,XvalYval,Zval,Uval,Vval and Wval values for that missile. 

global ro
global A
global g
dt = 0.005; 
g = 9.81;
ro = 1.2;
A = pi*(0.2)^2;
  
 load('terrain.mat');
 Vmag = 0;
 t = 0;
 T(1) = t;
 U(1) = 0;  %the missile is initially at rest at t = 0; So the velocity is 0
 V(1) = 0;
 W(1) = 0;
 X(1) = X0; %X0(n)
 Y(1) = Y0;
 Z(1) = Z0;
 n = 1;
 h = interp2(x_terrain, y_terrain, h_terrain,X(1), Y(1));

  while (Z(n) >= h)
%   for n = 1:100
        
       [Thx, Thy, Thz] = thrust(t, Thmag0, theta, phi, Tburn, U(n), V(n), W(n)); 
       Vmag = (U(n)^2 + V(n)^2 + W(n)^2)^(1/2);
       Thmag = (Thx^2 + Thy^2 + Thz^2)^(1/2);
       m = mass(t, m0, mf, Tburn); 
       Cd = drag_coeff(Vmag);
       U(n+1) = U(n) + (Thx/m - (Cd*ro*A/(2*m))*(U(n)*(U(n)^2+V(n)^2+W(n)^2)^(1/2)))*dt;
       V(n+1) = V(n) + (Thy/m - (Cd*ro*A/(2*m)*(V(n)*(U(n)^2+V(n)^2+W(n)^2)^(1/2))))*dt;
       W(n+1) = W(n) + (Thz/m - (Cd*ro*A/(2*m))*(W(n)*(U(n)^2+V(n)^2+W(n)^2)^(1/2)) - g)*dt;
       X(n+1) = X(n) + U(n+1)*dt;
       Y(n+1) = Y(n) + V(n+1)*dt;
       Z(n+1) = Z(n) + W(n+1)*dt;
       h = interp2(x_terrain, y_terrain, h_terrain, ...
                   X(end), Y(end));
       t = t + dt;
       T(n+1) = t;
       n = n+1  ;
  end
  Tval = T;
  Xval = X;
  Yval = Y;
  Zval = Z;
  Uval = U;
  Vval = V;
  Wval = W;
end  

































