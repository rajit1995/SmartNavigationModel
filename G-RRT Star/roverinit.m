function Rover= roverinit(RRTState)
Rover.PointA = RRTState.PointA;
Rover.PointB = RRTState.PointB;
Rover.Ref_threshold = norm(Rover.PointA - Rover.PointB);
Rover.Length = 0.35;
Rover.Width = 0.2488;
Rover.n=2;
% Rover.LoSRadius = Rover.n*Rover.Length;
Rover.Radius = 0.5*norm([Rover.Length Rover.Width]);
% Rover.LoSRadius = Rover.n*(Rover.Length+Rover.Radius);
Rover.LoSRadius = Rover.n*(Rover.Length);
Rover.waypoints = RRTState.finalpathvertices(:,1:2);
Rover.RotationTheta = pi/2;
Rover.Dimensions = RRTState.Dimensions;
Rover.Obstacles = RRTState.Obstacles;

r = randi([3,size(Rover.waypoints,1)-2]);

new_obstacle_center = Rover.waypoints(r,:);
% new_obstacle_center =  [11,15];
theta = 3.6;
theta_rad = theta*pi/180;
rnd =  0.5 + (2-0.5)*rand();
new_obstacle_x = zeros(1,100);
new_obstacle_y = zeros(1,100);
for j=1:100
    new_obstacle_x(j) =  (new_obstacle_center(1) + rnd*cos(j*theta_rad));
    new_obstacle_y(j) =  (new_obstacle_center(2) + rnd*sin(j*theta_rad));
end

Rover.Obstacles.X = [Rover.Obstacles.X;new_obstacle_x];
Rover.Obstacles.Y = [Rover.Obstacles.Y;new_obstacle_y];
Rover.Obstacles.Centers = [Rover.Obstacles.Centers;new_obstacle_center];
numObstacles = size(Rover.Obstacles.Centers,1);
Rover.Obstacles.Number = numObstacles;

 for i = 1:Rover.Obstacles.Number
    for j = 1 : size(Rover.Obstacles.X(i,:),2)
           if Rover.Obstacles.X(i,j) < Rover.Obstacles.Centers(i,1)
               Rover.Obstacles.X1(i,j) = Rover.Obstacles.X(i,j)-Rover.Radius;
           elseif Rover.Obstacles.X(i,j) > Rover.Obstacles.Centers(i,1)
               Rover.Obstacles.X1(i,j) = Rover.Obstacles.X(i,j)+Rover.Radius;
           else
               Rover.Obstacles.X1(i,j) = Rover.Obstacles.X(i,j);
           end
    end
        for j = 1 : size(Rover.Obstacles.Y(i,:),2)
           if Rover.Obstacles.Y(i,j) < Rover.Obstacles.Centers(i,2)
               Rover.Obstacles.Y1(i,j) = Rover.Obstacles.Y(i,j)-Rover.Radius;
           elseif Rover.Obstacles.Y(i,j) > Rover.Obstacles.Centers(i,2)
               Rover.Obstacles.Y1(i,j) = Rover.Obstacles.Y(i,j)+Rover.Radius;
           else
               Rover.Obstacles.Y1(i,j) = Rover.Obstacles.Y(i,j);
           end
        end

  end



Rover.StepSize =RRTState.StepSize;
Rover.Kpu = 15;
Rover.Kiu = 30;
Rover.kdh = 0.05;
Rover.Kph = 20;
Rover.Kih = 0.1;
Rover.kdu = 0.01;
Rover.dt = 0.005;
% Rover.dt = 0.01;
Rover.obstactalert = 0;
Rover.e_u_1 = [0 0];
Rover.u_sur =[0 0];
Rover.u_sur_1 =[0 0];
% Rover.pos_curr = Rover.waypoints(1,1:2);
Rover.pos_curr = Rover.waypoints(1,1:2);
Rover.pos_des =[0 0];
Rover.pos_des_1 =[0 0];
Rover.prev_wayPoint = Rover.waypoints(1,1:2);
Rover.Travel = [Rover.pos_curr];
Rover.counter = 1;
Rover.RadiusAcc = 0.4;
Rover.theta = 10;
Rover.theta_rad = Rover.theta*pi/180;
Rover.wpacc_ind = 0;
Rover.obst =[];
for j=1 : size(Rover.waypoints,1)
    ind =0;
            for i = 1:RRTState.Obstacles.Number
            [d,~,~] = p_poly_dist(Rover.waypoints(j,1),Rover.waypoints(j,2), Rover.Obstacles.X(i,:), Rover.Obstacles.Y(i,:)); 
            if d < 2*Rover.Radius 
                ind = ind +1;

            end

            end
              Rover.waypoints(j,3) = ind;
end
Rover.wp_num = size(Rover.waypoints,1);
end