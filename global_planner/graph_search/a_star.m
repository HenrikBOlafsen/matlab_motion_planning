function [path, goal_reached, cost, EXPAND] = a_star(map, start, goal)
% @file: a_star.m
% @breif: A* motion planning
% @paper: A Formal Basis for the heuristic Determination of Minimum Cost Paths
% @author: Winter
% @update: 2023.7.13

%
%   == OPEN and CLOSED ==
%   [x, y, g, h, px, py]
%   =====================
%

% initialize
OPEN = [];
CLOSED = [];
EXPAND = [];

cost = 0;
goal_reached = false;
motion = [-1, -1, sqrt(2); ...
    0, -1, 1; ...
    1, -1, sqrt(2); ...
    -1, 0, 1; ...
    1, 0, 1; ...
    -1, 1, sqrt(2); ...
    0, 1, 1; ...
    1, 1, sqrt(2)];

% Define the wind vector (e.g., wind blowing to the right with speed 1)
wind = [0, 1];  % Wind in the x-direction only (blowing right)

% Define a scaling factor for wind influence
alpha = 10;  % Adjust this value to control the effect of wind

% Initialize the new motion array (same size as motion)
new_motion = motion;

% Loop through each movement direction
for i = 1:size(motion, 1)
    % Extract the direction (first two elements of the motion row)
    direction = motion(i, 1:2);
    
    % Compute the dot product between the wind and the direction vector
    dot_product = dot(wind, direction);
    
    % Adjust the cost using a formula that prevents negative values
    wind_influence = 1 - alpha * dot_product;  % May become negative
    wind_influence = max(wind_influence, 0.5); % Bound the minimum cost factor to 0.5 (or some positive value)
    
    % Calculate the new cost
    new_cost = motion(i, 3) * wind_influence;
    
    % Update the new motion array with the modified cost
    new_motion(i, 3) = new_cost;
end

motion = new_motion;

motion_num = size(motion, 1);

node_s = [start, 0, h(start, goal), start];
OPEN = [OPEN; node_s];

while ~isempty(OPEN)
    % pop
    f = OPEN(:, 3) + OPEN(:, 4);
    [~, index] = min(f);
    cur_node = OPEN(index, :);
    OPEN(index, :) = [];

    % exists in CLOSED set
    if loc_list(cur_node, CLOSED, [1, 2])
        continue
    end

    % update expand zone
    if ~loc_list(cur_node, EXPAND, [1, 2])
        EXPAND = [EXPAND; cur_node(1:2)];
    end

    % goal found
    if cur_node(1) == goal(1) && cur_node(2) == goal(2)
        CLOSED = [cur_node; CLOSED];
        goal_reached = true;
        cost = cur_node(3);
        break
    end

    % explore neighbors
    for i = 1:motion_num
        node_n = [
            cur_node(1) + motion(i, 1), ...
            cur_node(2) + motion(i, 2), ...
            cur_node(3) + motion(i, 3), ...
            0, ...
            cur_node(1), cur_node(2)];
        node_n(4) = h(node_n(1:2), goal);

        % exists in CLOSED set
        if loc_list(node_n, CLOSED, [1, 2])
            continue
        end

        % obstacle
        if map(node_n(1), node_n(2)) == 2
            continue
        end

        % update OPEN set
        OPEN = [OPEN; node_n];
    end
    CLOSED = [cur_node; CLOSED];
end

% extract path
path = extract_path(CLOSED, start);
end

%%
function h_val = h(node, goal)
% @breif: heuristic function (Manhattan distance)
h_val = abs(node(1) - goal(1)) + abs(node(2) - goal(2));
end

function index = loc_list(node, list, range)
% @breif: locate the node in given list
num = size(list);
index = 0;

if ~num(1)
    return
else
    for i = 1:num(1)
        if isequal(node(range), list(i, range))
            index = i;
            return
        end
    end
end
end

function path = extract_path(close, start)
% @breif: Extract the path based on the CLOSED set.
path = [];
closeNum = size(close, 1);
index = 1;

while 1
    path = [path; close(index, 1:2)];

    if isequal(close(index, 1:2), start)
        break
    end

    for i = 1:closeNum
        if isequal(close(i, 1:2), close(index, 5:6))
            index = i;
            break
        end
    end
end
end
