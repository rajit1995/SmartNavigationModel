function RRTState1 = getqnear2(RRTState1)
   min = 1000000; 
    for i=1:size(RRTState1.pathvertices,1)
            dist = norm(RRTState1.pathvertices(i,1:2) - RRTState1.PointB);
            if dist<min
                min = dist;
                RRTState1.q_near = RRTState1.pathvertices(i,1:2);
                RRTState1.nearidx = i;
            else 
                continue;
            end
     end
end
