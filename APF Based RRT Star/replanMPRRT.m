function [Rover,RRTState1] = replanMPRRT(RRTState, Rover,n)
disp('Replanning')
RRTState1 = RRTState;
RRTState1.Obstacles = Rover.Obstacles;
RRTState1.PointA = Rover.prev_wayPoint;
RRTState1.GoalReachInd =0;
waypoint1 = Rover.waypoints(1:n-1,1:2);
RRTState1.Branches1 = [1,1,0];
RRTState1.pathvertices = [RRTState1.PointA,0];
RRTState1.distances = [];
RRTState1.neighboringIndices = [];
RRTState1.branchDetailsArray = [];
RRTState1.path = [];
RRTState1.pathBranches = [];
RRTState1.finalpathvertices = [];

%Checking for the next available Waypoint
for m = n:size(Rover.waypoints,1)
    next_wayPoint = Rover.waypoints(m,1:2); 
     for j = 1:Rover.Obstacles.Number
            [in(j),out(j)] = inpolygon(next_wayPoint(1), next_wayPoint(2), Rover.Obstacles.X1(j,:), Rover.Obstacles.Y1(j,:));
            indicator = [in,out];
            sum_ind = sum(indicator);
      end
      if sum_ind == 0
          % Rover.waypoints(i,:) = [];
            next_available_wp_idx = m;
            break;
        
      end
end
waypoint2 = Rover.waypoints(next_available_wp_idx:size(Rover.waypoints,1),1:2);
RRTState1.PointB = Rover.waypoints(next_available_wp_idx,1:2);
RRTState1.Destination = RRTState1.PointB;
%%************************************************************************************
    RRTState1.q_new = RRTState1.PointA;
    RRTState1.iteration.count = 1;
    RRTState1.q_near = RRTState1.PointA;
    %rng(2);
    while RRTState1.iteration.count < RRTState1.iteration.max
        % visualrrt(RRTState);
     
        RRTState1.q_new = 3*rand()*norm(RRTState1.PointA - RRTState1.PointB)*[cos(2*pi*rand()), sin(2*pi*rand())] + RRTState1.PointA;        
        RRTState1.distances = sqrt(sum((RRTState1.pathvertices(:,1:2) - RRTState1.q_new).^2, 2));
        [~,RRTState1.nearidx] = min(RRTState1.distances);
        
        RRTState1.q_near = RRTState1.pathvertices(RRTState1.nearidx,1:2);
       
           RRTState1 = getqnew2(RRTState1);
        in = zeros(1,RRTState1.Obstacles.Number);
        out = zeros(1,RRTState1.Obstacles.Number);
        for i = 1:RRTState1.Obstacles.Number
            %RRTState.q_new(1)
            %RRTState.q_new(2)
            % RRTState.Obstacles.X(i,:)
            % RRTState.Obstacles.Y(i,:)
            [in(i),out(i)] = inpolygon(RRTState1.q_new(1), RRTState1.q_new(2), RRTState1.Obstacles.X1(i,:), RRTState1.Obstacles.Y1(i,:));
            indicator = [in,out];
            sum_ind = sum(indicator);
        end

        if sum_ind == 0  
             sz = size(RRTState1.Branches1,1);
                   
                    RRTState1.cost_ref = norm(RRTState1.pathvertices(RRTState1.nearidx,1:2)-RRTState1.q_new);
                    RRTState1.new_node_cost = RRTState1.pathvertices(RRTState1.nearidx,3) + RRTState1.cost_ref;
                    if size(RRTState1.pathvertices,1)>=1
                        
                        RRTState1 = rewireRRT2(RRTState1,sz);
                    end
               
        else
            
           continue;
            
        end


                if norm(RRTState1.q_new-RRTState1.PointB) <= RRTState1.Threshold
                    RRTState1.GoalReachInd =1;
                   RRTState1.Final.Iterations = RRTState1.iteration.count;
                   
                   
                    
                end
        
        RRTState1.iteration.count = RRTState1.iteration.count+1;
        

%disp( RRTState1.iteration.count);
    end
      if RRTState1.GoalReachInd ==1
                    sz = size(RRTState1.Branches1,1);
                   RRTState1.q_new=RRTState1.PointB;
                   RRTState1.distances = sqrt(sum((RRTState1.pathvertices(:,1:2) - RRTState1.q_new).^2, 2));
                    [~,RRTState1.nearidx] = min(RRTState1.distances);
        
                    RRTState1.cost_ref = norm(RRTState1.pathvertices(RRTState1.nearidx,1:2)-RRTState1.q_new);
                    RRTState1.new_node_cost = RRTState1.pathvertices(RRTState1.nearidx,3) + RRTState1.cost_ref;
                    RRTState1 = rewireRRT2(RRTState1,sz);
                    %RRTState1.pathvertices(size(RRTState1.pathvertices,1)+1,:) = [RRTState1.PointB,]
                    RRTState1 = getpath2(RRTState1);
                    RRTState1 = getfinalpathvertices2(RRTState1);
                    RRTState1.Final.dist = hypot(diff(RRTState1.finalpathvertices(:,1)), diff(RRTState1.finalpathvertices(:,2)))  ;  
                    RRTState1.Final.dist_total = sum(RRTState1.Final.dist);
                    % RRTState1.plotfinalpath = 1;
                      % visualrrt(RRTState);

 
      end
Rover.waypoints =[];  
Rover.waypoints = [waypoint1;RRTState1.finalpathvertices(2:size(RRTState1.finalpathvertices,1),1:2);waypoint2];
Rover.wp_num = size(Rover.waypoints,1);
Rover.next_wayPoint = Rover.waypoints(n,1:2); 
% disp(Rover.waypoints);
% disp(waypoint1);
% disp(waypoint2);
% disp(Rover.next_wayPoint);
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
% disp(Rover.waypoints);
% disp(Rover.prev_wayPoint);
% disp(Rover.next_wayPoint);
%%****************************************************************************************
end
