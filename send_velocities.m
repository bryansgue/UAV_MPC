function  send_velocities(robot, velmsg, vd)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Send desired velocities to the robot Linear
vx = vd(1);
vy = vd(2);
vz = vd(3);

% Send desired velocities to the robot angular
wx = 0;
wy = 0;
wz = vd(4);

velmsg.Twist.Linear.X = vx;
velmsg.Twist.Linear.Y = vy;
velmsg.Twist.Linear.Z = vz;

velmsg.Twist.Angular.X = wx;
velmsg.Twist.Angular.Y = wy;
velmsg.Twist.Angular.Z = wz;


send(robot,velmsg);
end

