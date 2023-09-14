function [position,orientacion,v_lineal,v_angular,quaternio] = odometryUAV(odomSub)
% Read odometry values from ros
  odomdata = receive(odomSub,0.2);  %(the second argument is a time-out in seconds).
pose = odomdata.Pose.Pose.Position;
vel_lineal = odomdata.Twist.Twist.Linear;
vel_angular = odomdata.Twist.Twist.Angular;
quat = odomdata.Pose.Pose.Orientation;
quaternio = [quat.W quat.X quat.Y quat.Z];

position = [pose.X;pose.Y;pose.Z];
orientacion_aux = (quat2eul(quaternio,"ZYX"))';
orientacion = [orientacion_aux(3);orientacion_aux(2);orientacion_aux(1)];
v_lineal = [vel_lineal.X;vel_lineal.Y;vel_lineal.Z];
v_angular = [vel_angular.X;vel_angular.Y;vel_angular.Z];
end

