function [x,y,z] = Rotation3D_MAAT(x,y,z,angle1,angle2,angle3)
%UNTITLED2 Summary of this function goes here



% Around X-axis:
xx = x;
yx = y*cos(angle1) - z*sin(angle1);
zx = y*sin(angle1) + z*cos(angle1);

% Around Y-axis:
xy = xx*cos(angle2) + zx*sin(angle2);
yy = yx;
zy = zx*cos(angle2) - xx*sin(angle2);

% Around Z-axis:
x = xy*cos(angle3) - yy*sin(angle3);
y = xy*sin(angle3) + yy*cos(angle3);
z = zy;


end

