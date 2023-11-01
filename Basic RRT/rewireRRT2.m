function RRTState1 = rewireRRT2(RRTState1,sz)
        RRTState1.new_branch = zeros(1,3);
        RRTState1.distances = sqrt(sum((RRTState1.pathvertices(:,1:2) - RRTState1.q_new).^2, 2));
        RRTState1.neighboringIndices = find(RRTState1.distances <= RRTState1.rwradius);
        
        vert_sz = size(RRTState1.pathvertices,1);
        RRTState1.new_branch = [RRTState1.nearidx,vert_sz,RRTState1.cost_ref];
     % Branch_ref = norm(RRTState.pathvertices(RRTState.neighboringIndices(i),1:2)-RRTState.q_new)
         Branch_ref = norm(RRTState1.pathvertices(RRTState1.nearidx,1:2)-RRTState1.q_new);
            for i=1:size(RRTState1.neighboringIndices)
            cost_new = RRTState1.pathvertices(RRTState1.neighboringIndices(i),3) +...
                norm(RRTState1.pathvertices(RRTState1.neighboringIndices(i),1:2)-RRTState1.q_new);
            Branch_ref = norm(RRTState1.pathvertices(RRTState1.neighboringIndices(i),1:2)-RRTState1.q_new);
            if cost_new <= RRTState1.new_node_cost
                %disp('Rewire')
                RRTState1.new_node_cost = cost_new;
                %
                % nearidx = RRTState.neighboringIndices(i,:);
                 RRTState1.nearidx = RRTState1.neighboringIndices(i,:);
            else
                continue;
            end 
            end
        
    RRTState1 = polyintersect2(RRTState1);
    if ~RRTState1.int_ind
     RRTState1.pathvertices(size(RRTState1.pathvertices,1)+1,:) =[RRTState1.q_new,RRTState1.new_node_cost];
    
     RRTState1.Branches1(size(RRTState1.Branches1,1)+1,:) = [RRTState1.nearidx,size(RRTState1.pathvertices,1),Branch_ref];
    
        
    end




end