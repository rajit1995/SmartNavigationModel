function RRTState1 = getpath2(RRTState1)
    numNodes = size(RRTState1.pathvertices, 1);
    startNode = 1;
    for i = 1:size(RRTState1.pathvertices, 1) 
        if RRTState1.pathvertices(i,1:2) == RRTState1.PointB
            endNode = i;
        end
    end

    tree = cell(1, numNodes);
    for i = 1:size(RRTState1.Branches1, 1)
        parentIdx = RRTState1.Branches1(i, 1);
        childIdx = RRTState1.Branches1(i, 2);
        cost = RRTState1.Branches1(i, 3); 
        
        tree{parentIdx} = [tree{parentIdx}; childIdx, cost];
        tree{childIdx} = [tree{childIdx}; parentIdx, cost]; % For undirected tree
    end

    % Check if start and end nodes are valid
    if startNode < 1 || startNode > numNodes || endNode < 1 || endNode > numNodes
        error('Invalid start or end node indices.');
    end

    % Initialize the queue and visited array
    queue = startNode;
    visited = false(1, numNodes);

    % Initialize parent array and branch details array to keep track of the path
    parent = zeros(1, numNodes);
    parent(startNode) = startNode;

    RRTState1.branchDetailsArray = cell(1, numNodes); % Store branch details for each node
    RRTState1.branchDetailsArray{startNode} = [];

    % Perform BFS traversal
    while ~isempty(queue)
        currentNode = queue(1);
        queue(1) = [];
        visited(currentNode) = true;

        % Check if the end node is reached
        if currentNode == endNode
            % Reconstruct the path from end to start
           currentNode = endNode;
           RRTState.path = currentNode;

            while currentNode ~= startNode
                currentNode = parent(currentNode);
                RRTState1.path = [currentNode, RRTState1.path];
            end
            RRTState1.path;
    RRTState1.pathBranches = [];
    for i = 1:length(RRTState1.path)-1
        currentNode = RRTState1.path(i);
        nextNode = RRTState1.path(i+1);

        [~, idx] = ismember([currentNode, nextNode], RRTState1.Branches1(:,1:2), 'rows');
        if idx > 0
            RRTState1.pathBranches = [RRTState1.pathBranches; RRTState1.Branches1(idx, :)];
        else
            error('Error: Branch details not found for the path.');
        end
    end
            return;
        end

        % Add unvisited neighbors to the queue
        neighbours = tree{currentNode};
        for i = 1:size(neighbours, 1)
            neighbour = neighbours(i, 1);
            if ~visited(neighbour)
                queue(end+1) = neighbour;
                visited(neighbour) = true;
                parent(neighbour) = currentNode;
                RRTState1.branchDetailsArray{neighbour} = [RRTState1.branchDetailsArray{currentNode}; currentNode, neighbour];
            end
        end
    end

    end