function T=eulerZYX2T(X,Y,Z,rotZ,rotY,rotX)
T=[cos(rotZ)*cos(rotY) cos(rotZ)*sin(rotY)*sin(rotX)-sin(rotZ)*cos(rotX) cos(rotZ)*sin(rotY)*cos(rotX)+sin(rotZ)*sin(rotX) X;
   sin(rotZ)*cos(rotY) sin(rotZ)*sin(rotY)*sin(rotX)+cos(rotZ)*cos(rotX) sin(rotZ)*sin(rotY)*cos(rotX)-cos(rotZ)*sin(rotX) Y;
   -sin(rotY)          cos(rotY)*sin(rotX)                               cos(rotY)*cos(rotX)                               Z;
    0 0 0 1];
end
