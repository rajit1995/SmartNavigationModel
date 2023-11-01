function rover_plot(Rover)
    hold on
    %create the free space
    qw{1} = fill([0, Rover.Dimensions.Width, Rover.Dimensions.Width, 0],...
        [0, 0, Rover.Dimensions.Length, Rover.Dimensions.Length],...
        [60, 125, 30]/255);


    %Plot the Obstacles
    for i = 1:Rover.Obstacles.Number
        qw{4} =    fill(Rover.Obstacles.X1(i,:), Rover.Obstacles.Y1(i,:),[0.7, 0.7,0.7]);
        if i==3
        qw{2} =     fill(Rover.Obstacles.X(i,:), Rover.Obstacles.Y(i,:), [20, 80, 160] / 255);
        else
        qw{3} =    fill(Rover.Obstacles.X(i,:), Rover.Obstacles.Y(i,:), [68, 35, 0] / 255);
        end
    end

    %Plot the area covered by the rover during the traversing
    for branchidx=1:size(Rover.Travel,1)     
    rectangle('Position',[Rover.Travel(branchidx,1)-Rover.Radius, Rover.Travel(branchidx,2)-Rover.Radius, 2*Rover.Radius, 2*Rover.Radius],'Curvature', [1,1], 'FaceColor','red','EdgeColor', 'none');

    end

    %Plot the waypoints
    qw{6} =  scatter(Rover.waypoints(:,1), Rover.waypoints(:,2), 'MarkerFaceColor', 'k');

    %mark the Start and End Point
    qw{7} =scatter(Rover.PointA(1), Rover.PointA(2), 'filled', 'MarkerFaceColor', '#A2142F');
    qw{8} =scatter(Rover.PointB(1), Rover.PointB(2), 'filled', 'MarkerFaceColor', '#A214EF');


    %Plot the Center of Mass the rover during the traversing
     qw{9} =plot([Rover.Travel(:,1)],  [Rover.Travel(:,2)],'y','LineWidth',1);


    %Plot the Next Waypoint with the Circle of Acceptance
    rectangle('Position',[Rover.next_wayPoint(1)-Rover.RadiusAcc, Rover.next_wayPoint(2)-Rover.RadiusAcc, 2*Rover.RadiusAcc, 2*Rover.RadiusAcc],'Curvature', [1,1], 'EdgeColor','#EDB120');
    qw{11} =scatter(Rover.next_wayPoint(1), Rover.next_wayPoint(2), 'filled', 'MarkerFaceColor', '#EDB120');
   
    %Plot The Rover
      rectangle('Position',[Rover.pos_curr(1)-Rover.Radius, Rover.pos_curr(2)-Rover.Radius, 2*Rover.Radius, 2*Rover.Radius],'Curvature', [1,1], 'FaceColor','w');
      dir = Rover.u_sur/norm(Rover.u_sur);
        %Direction of Rover
    plot([Rover.pos_curr(1),Rover.pos_curr(1)+dir(1)*Rover.Radius],[Rover.pos_curr(2), Rover.pos_curr(2)+dir(2)*Rover.Radius],'LineWidth', 0.3,'Color','k');
     
    %Plot the Line of Sight of Rover
    rectangle('Position',[Rover.pos_curr(1)-Rover.LoSRadius, Rover.pos_curr(2)-Rover.LoSRadius, 2*Rover.LoSRadius, 2*Rover.LoSRadius],'Curvature', [1,1], 'EdgeColor','c');
    qw{13} =line(NaN,NaN,'LineWidth',1,'Color','c'); 

   
    axis equal;

    % Set plot limits based on field dimensions
    xlim([0, Rover.Dimensions.Width]);
    ylim([0, Rover.Dimensions.Length]);
    xlabel('x axis')
    ylabel('y axis')
    
    title('Rover model : APF RRT STAR')

 qw{5} = line(NaN,NaN,'LineWidth',5,'Color','r');
 qw{10} = line(NaN,NaN,'LineWidth',1,'Color','#EDB120'); 
 qw{12} =scatter(NaN,NaN,'MarkerEdgeColor','k','MarkerFaceColor','w');  
 qw{13} =line(NaN,NaN,'LineWidth',1,'Color','c'); 

 legend([qw{:}],{'Free Space','Lake','Obstacles', 'Prohibited Zone for Nodes',...
     'Traversed Area covered by Rover', 'WayPoints', 'Start Point', 'End Point',...
     'Trajectory of Center Of Mass of Rover', 'Circle of Acceptance for Next Waypoint',...
     'Next Waypoint', 'Planetary Rover', 'LoS Circle of Rover'}, 'location', 'northeastoutside')

    hold off
  
end