function [V] = T2eulerZYX(T)

X = T(1,4);
Y = T(2,4);
Z = T(3,4);
O(2)=atan2(-T(3,1),sqrt(T(1,1)^2+T(2,1)^2));
if (abs(abs(O(2))-90)<0.00001)
    O(1)=0;
    O(3)=O(2)/abs(O(2))*atan2(T(1,2),T(2,2));
else
    O(1)=atan2(T(2,1)/cos(O(2)),T(1,1)/cos(O(2)));
    O(3)=atan2(T(3,2)/cos(O(2)),T(3,3)/cos(O(2)));
end
r = O(3);
p = O(2);
y = O(1);
% p = atan2(-T(3,1), sqrt( T(3,2)^2 + T(3,3)^2 ));
% y = atan2(T(2,1)/cos(p),T(1,1)/cos(p));
% r = atan2(T(3,2)/cos(p),T(3,3)/cos(p));
%V = [X , Y, Z, rad2deg(r), rad2deg(p), rad2deg(y)];
V = [X , Y, Z, r, p, y];
