function Rover = losapnav(Rover,RRTState)
disp('Rover Navigating')

 % disp(Rover.waypoints);
 i=2;
 % for i = 2:size(Rover.waypoints,1)
 while i <=    Rover.wp_num
     % disp(i);
      Rover.next_wayPoint = Rover.waypoints(i,1:2);
      if i==2
          visualrover(Rover);
            filename = 'RoverModel_new_obstacle.fig';                                                                                                                    
            saveas(1,filename);
      end
      % disp(Rover.prev_wayPoint);
      % disp(Rover.next_wayPoint);
      in = zeros(1:Rover.Obstacles.Number);
      out = zeros(1:Rover.Obstacles.Number);
      for j = 1:Rover.Obstacles.Number
            [in(j),out(j)] = inpolygon(Rover.next_wayPoint(1), Rover.next_wayPoint(2), Rover.Obstacles.X1(j,:), Rover.Obstacles.Y1(j,:));
            indicator = [in,out];      
            B = [Rover.prev_wayPoint(:,1),Rover.next_wayPoint(:,1)];
            C = [Rover.prev_wayPoint(:,2),Rover.next_wayPoint(:,2)];
            int_ind =[];
            [x_i,y_i] = polyxpoly(B,C,Rover.Obstacles.X1(j,:), Rover.Obstacles.Y1(j,:));
            int_ind = [int_ind,x_i,y_i];
             sum_ind = sum(indicator);
             sum_ind = sum(int_ind) + sum_ind;
      end
    


      if sum_ind > 0
          disp('New Obstacle Detected')
          % % % Rover.waypoints(i,:) = [];
          % % % % Rover.next_wayPoint = Rover.waypoints(i,1:2); 
          % % % i = i-1;
          % % % continue;
          Rover = smartalgoselect(Rover);
          % disp(Rover.theta_ref);
          % short_dist = norm(Rover.pos_curr - Rover.PointB)
          % best_dist = norm(Rover.PointB - Rover.PointA)
          if norm(Rover.pos_curr - Rover.PointA) <= 0.15*norm(Rover.PointA - Rover.PointB)
              disp('Multi-Partite RRT Selected, if');
              [Rover,RRTState1] = replanMPRRT(RRTState, Rover,i);
          elseif norm(Rover.pos_curr - Rover.PointB) > 0.15*norm(Rover.PointA - Rover.PointB) && Rover.theta_ref >= 180 
                  disp('Dynamic RRT Selected, elseif 1');
                  [Rover,RRTState1] = replanERRT(RRTState, Rover,i);
          elseif norm(Rover.pos_curr - Rover.PointB) > 0.15*norm(Rover.PointA - Rover.PointB) && Rover.theta_ref < 180
                  disp('Multi-Partite RRT Selected, elseif 2');
                  [Rover,RRTState1] = replanMPRRT(RRTState, Rover,i);
          else 
                  disp('Dynamic RRT Selected, else');
                  [Rover,RRTState1] = replanERRT(RRTState, Rover,i);              
          end

        % [Rover,RRTState1] = replanERRT(RRTState, Rover,i);
        disp('Replanned')
        visualrover(Rover);
        filename = ['Replanned_Path.fig'];                                                                                                                    
        saveas(1,filename)
      end
     
     

    while Rover.wpacc_ind ~=1

        Rover.counter = Rover.counter+1;
      
      if i==size(Rover.waypoints,1)
          Rover.RadiusAcc = 0.1;
          if norm(Rover.pos_curr - Rover.next_wayPoint) >= Rover.n*(Rover.Length)
              Rover.LoSRadius = Rover.LoSRadius;
          else
              Rover.LoSRadius = norm(Rover.pos_curr - Rover.next_wayPoint);
          end
      else
          if Rover.waypoints(i,3) > 0
              if norm(Rover.prev_wayPoint-Rover.next_wayPoint) <= Rover.StepSize
                  Rover.RadiusAcc = norm(Rover.prev_wayPoint-Rover.next_wayPoint);
              else
                  Rover.RadiusAcc = norm(Rover.prev_wayPoint-Rover.next_wayPoint)*0.2;
              end
          else 
              Rover.RadiusAcc = norm(Rover.prev_wayPoint-Rover.next_wayPoint)*0.3;
          end

      end
      
     for j=1:36
       Rover.LosCircle(j,:) = [(Rover.pos_curr(1) + Rover.LoSRadius*cos(j*Rover.theta_rad)) (Rover.pos_curr(2) + Rover.LoSRadius*sin(j*Rover.theta_rad))];
       
    end

    B = [Rover.prev_wayPoint(:,1),Rover.next_wayPoint(:,1)];
    C = [Rover.prev_wayPoint(:,2),Rover.next_wayPoint(:,2)];
    [xi,yi] = polyxpoly(B,C,Rover.LosCircle(:,1),Rover.LosCircle(:,2));
        if size(xi,1) > 1
            if norm([xi(1) yi(1)] - Rover.next_wayPoint) < norm([xi(2) yi(2)] - Rover.next_wayPoint)
                Rover.pos_des = [xi(1) yi(1)];
            else
                    Rover.pos_des = [xi(2) yi(2)];
            end
        elseif size(xi,1) == 1
            Rover.pos_des = [xi yi];
           
        else 
            Rover.pos_des = Rover.next_wayPoint;
        end
        if norm(Rover.pos_des_1 - Rover.next_wayPoint) < norm(Rover.pos_des - Rover.next_wayPoint)  && Rover.obstactalert ~=1
            Rover.pos_des = Rover.next_wayPoint;
        end
     
        if norm(Rover.pos_curr - Rover.next_wayPoint) <= Rover.RadiusAcc 
             Rover.wpacc_ind = 1;
        else
             Rover.wpacc_ind = 0;
             
        
         Rover.e_u = Rover.pos_des - Rover.pos_curr;
        
         Rover.u_sur = Rover.Kpu*Rover.e_u + Rover.Kiu*(Rover.e_u+Rover.e_u_1)*Rover.dt/2 + Rover.kdu*(Rover.e_u - Rover.e_u_1)/Rover.dt;
         % u_sur_bkp = Rover.u_sur;
         % head_angle = acos(sum(Rover.u_sur.*Rover.u_sur_1)/(norm(Rover.u_sur)*norm(Rover.u_sur_1)));
         % if head_angle > pi/3 && Rover.obstactalert ~= 1
         %     head_angle_new1 = -1*head_angle;
         % 
         %     Rover.u_sur1 =  transpose([cos(head_angle_new1) -sin(head_angle_new1);sin(head_angle_new1) cos(head_angle_new1)]*transpose(Rover.u_sur));
         %     if head_angle <0 
         %         head_angle_new = -pi/12;
         %     else 
         %        head_angle_new = pi/12;
         %     end
         %     Rover.u_sur = transpose([cos(head_angle_new) -sin(head_angle_new);sin(head_angle_new) cos(head_angle_new)]*transpose(Rover.u_sur1));
         % else
         %    Rover.u_sur = u_sur_bkp;
         % end


        Rover.a = (Rover.u_sur - Rover.u_sur_1)/Rover.dt;
         Rover.disp = Rover.u_sur_1*Rover.dt + 0.5*Rover.a*Rover.dt*Rover.dt;
         Rover = obstacledetect(Rover) ;
        %  Rover.obst =  [Rover.obst;Rover.poly_ind];
        % Rover.pos_curr = Rover.pos_curr + Rover.disp;
        % % pos_curr = Rover.pos_curr;
        % Rover.Travel = [Rover.Travel;Rover.pos_curr];
         Rover.e_u_1 = Rover.e_u;
         Rover.u_sur_1 = Rover.u_sur;
         Rover.pos_des_1 =Rover.pos_des;
         visualrover(Rover);
        end
    end
       Rover.prev_wayPoint = Rover.next_wayPoint;
       Rover.wpacc_ind = 0;
       i = i+1;
    
  end
  disp('Reached');
    Rover.dist1 = hypot(diff(Rover.Travel(:,1)), diff(Rover.Travel(:,2)))  ;  
    Rover.dist_total1 = sum(Rover.dist1); 
end
