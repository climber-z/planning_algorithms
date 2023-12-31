function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    %OPEN(OPEN_COUNT,1)=0;  % Mistake? This node has already been on the list.
    OPEN(OPEN_COUNT,1)=1; % So changed into 1.
%     CLOSED_COUNT=CLOSED_COUNT+1;
%     CLOSED(CLOSED_COUNT,1)=xNode;
%     CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while(1) %you have to dicide the Conditions for while loop exit 
        if isempty(OPEN)
            disp("No Path.");
            break
        end
        min_i = min_fn(OPEN, OPEN_COUNT, xTarget, yTarget); % min_i is the least cost node's index
        if (OPEN(min_i, 2) == xTarget) && (OPEN(min_i, 3) == yTarget)
            CLOSED_COUNT = CLOSED_COUNT + 1;
            CLOSED(CLOSED_COUNT, 1) = OPEN(min_i, 2);
            CLOSED(CLOSED_COUNT, 2) = OPEN(min_i, 3);
            disp("Find Path.");
            NoPath = 0;
            break;
        end
        xParent = OPEN(min_i,2);
        yParent = OPEN(min_i,3);
        gnParent = OPEN(min_i,7);
        % expand neighbor nodes
        OPEN(min_i, 1) = 0; % Mark that expanded node is not on the list
        % OPEN(min_i, :) = []; % Delete expanded node from open list
        CLOSED_COUNT = CLOSED_COUNT + 1;
        CLOSED(CLOSED_COUNT, 1) = OPEN(min_i, 2);
        CLOSED(CLOSED_COUNT, 2) = OPEN(min_i, 3);
        neighbor = [xParent-1,yParent-1; xParent-1,yParent; xParent-1,yParent+1;
            xParent, yParent-1; xParent, yParent+1;
            xParent+1,yParent-1; xParent+1,yParent; xParent+1,yParent+1];
        for i=1:size(neighbor, 1) % Traverse all neighbor nodes
            nx = neighbor(i,1);
            ny = neighbor(i,2);
            if (nx > 0) && (nx < MAX_X) && (ny > 0) && (ny < MAX_Y)
                if (MAP(nx, ny) ~= -1) % Choose the nodes who are in boundary but not obstacles
                    n_i = node_index(OPEN, nx, ny);
                    if n_i == 0 % If the node is newly discovered, then add to the open list
                        hn = distance(nx,ny,xTarget,yTarget);
                        gn = gnParent + distance(xParent,yParent,nx,ny);
                        fn = hn + gn;
                        OPEN_COUNT = OPEN_COUNT + 1;
                        OPEN(OPEN_COUNT, :) = insert_open(nx, ny, xParent, yParent, hn, gn, fn);
                    else % If the node is already on the list, update the least gn and then fn
                        gn = gnParent + distance(xParent,yParent,nx,ny);
                        if OPEN(n_i, 7) > gn
                            OPEN(n_i, 7) = gn;
                            OPEN(n_i, 8) = gn + OPEN(n_i, 6);
                            OPEN(n_i, 4) = xParent;
                            OPEN(n_i, 5) = yParent;
                        end
                    end
                end
            end
        end        
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
   if NoPath == 0
       x = xTarget;
       y = yTarget;
       path_count = 0;
       while (x ~= xStart || y ~= yStart)
           ni = node_index(OPEN, x, y);
           x = OPEN(ni, 4);
           y = OPEN(ni, 5);
           path_count = path_count + 1;
           path(path_count, 1) = x - 0.5;
           path(path_count, 2) = y - 0.5;
   end
end
