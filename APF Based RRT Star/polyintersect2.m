function RRTState1 = polyintersect2(RRTState1)
B = [RRTState1.pathvertices(RRTState1.nearidx,1),RRTState1.q_new(1)];
C = [RRTState1.pathvertices(RRTState1.nearidx,2),RRTState1.q_new(2)];
int_ind =0;
    for i=1:RRTState1.Obstacles.Number
    [xi,yi] = polyxpoly(B,C,RRTState1.Obstacles.X1(i,:), RRTState1.Obstacles.Y1(i,:));
    sum_ind = sum([xi,yi]);
    int_ind = int_ind+sum_ind;
    end
    
RRTState1.int_ind = int_ind;
end