
colorInside = [0.8,0.6,0.4];
iterations =1000;
Length=25;
Width = 25;
% numObstacles = 5;
obstacleCenters=[22.923178545696597,17.857236168782418;...
    14.407472476412247,13.988390759430509;...
    15.364193058888265,15.352357565818371;...
    4.712130275488792,15.615449471544114;...
    7.085026150145800,8.238076132920797];
numObstacles = size(obstacleCenters,1);
new_obstacle_center = [6,11];
theta = 3.6;
theta_rad = theta*pi/180;
for j=1:100
    new_obstacle_x(j) =  (new_obstacle_center(1) + 0.5*cos(j*theta_rad));
    new_obstacle_y(j) =  (new_obstacle_center(2) + 0.5*sin(j*theta_rad));
end

pointA= [4,4];
pointB = [20,20];
q_near = pointA;
q_new = pointA;
r_near = 2;
iternum =1;
RRTState.q_near_count=0;