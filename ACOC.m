clc; clear all; close all;

%%
% Ant Colony Optimization Clustering (ACOC)
% Last Updated: 5/2/2018
% Author: Garrett Kaiser
% GNU GENERAL PUBLIC LICENSE Version 3
% See the License document for further information

%% Constants Parameters
s = 8;
alpha = .35;
kp = 0.2;
kd = 0.05;
phimax = 5;
phimin = 0.1;
pheromoneIncerement = .5;
pheromoneDecrement = .99;
nSize = 10;    % neighborhood size
max_d = 0.001; % max normalized density
c = 10;        % constant for convergence control

%% Initialize Data

% optional data set with 4 classes
% data = [normrnd(4,1.2,[50 1]),normrnd(4,1.2,[50 1]);
%         normrnd(-4,1.2,[50 1]),normrnd(4,1.2,[50 1]);
%         normrnd(-4,1.2,[50 1]),normrnd(-4,1.2,[50 1]);
%         normrnd(4,1.2,[50 1]),normrnd(-4,1.2,[50 1])];
% numDataPoints = length(data);
% classA = 1:25;
% classB = 26:50;
% classC = 51:75;
% classD = 76:100;
% plot(data(:,1),data(:,2),'*');

% data set with 2 classes
% the data is a set of 10 randomly created values that represent arbitrary node attributes
% the vector of 10 values can represent any desired node attributes
data = [normrnd(5,0.25,[200 10]);normrnd(-5,0.25,[200 10])];
numDataPoints = length(data);
classA = 1:200;
classB = 201:400;

gridSize = 400;
gridDim = [1,gridSize];
gridMat = zeros(gridSize);
foodMat = zeros(gridSize);
antMat = zeros(gridSize);
pheromoneMat = phimin*ones(gridSize);

idx = randperm(numel(gridMat), numDataPoints);
[row_sub,col_sub] = ind2sub(size(gridMat), idx);
sub_mtx = [row_sub(:), col_sub(:)];
for i = 1:numDataPoints
    food(i,1) = {data(i,:)};    % each food object is assigned 10 data attributes
    food(i,2) = {sub_mtx(i,:)}; % location of food object
end
for i = 1:length(sub_mtx)
    foodMat(sub_mtx(i,1),sub_mtx(i,2)) = i;
end

antCount = 400;
idx = randperm(numel(gridMat), antCount);
[row_sub,col_sub] = ind2sub(size(gridMat), idx);
sub_mtx = [row_sub(:), col_sub(:)];
for i = 1:antCount
    ants(i,1) = {sub_mtx(i,:)};
    ants(i,2) = {0};
end
for i = 1:length(sub_mtx)
    antMat(sub_mtx(i,1),sub_mtx(i,2)) = i;
end

%% Plot Initial Locations

getcol = @(M, col) M(:,col);

figure(1);
hold on;
grid off;
axis off;
plot(getcol(cell2mat(food(classA,2)),1),getcol(cell2mat(food(classA,2)),2),'co');
plot(getcol(cell2mat(food(classB,2)),1),getcol(cell2mat(food(classB,2)),2),'mo');
% plot(getcol(cell2mat(food(classC,2)),1),getcol(cell2mat(food(classC,2)),2),'yo');
% plot(getcol(cell2mat(food(classD,2)),1),getcol(cell2mat(food(classD,2)),2),'ko');

figure(2);
hold on;
grid off;
axis off;
plot(getcol(cell2mat(ants(:,1)),1),getcol(cell2mat(ants(:,1)),2),'r*');
plot(getcol(cell2mat(food(classA,2)),1),getcol(cell2mat(food(classA,2)),2),'co');
plot(getcol(cell2mat(food(classB,2)),1),getcol(cell2mat(food(classB,2)),2),'mo');
% plot(getcol(cell2mat(food(classC,2)),1),getcol(cell2mat(food(classC,2)),2),'yo');
% plot(getcol(cell2mat(food(classD,2)),1),getcol(cell2mat(food(classD,2)),2),'ko');

%% Simulate ACO Clustering
for iter = 1:8000
    for antId = 1:antCount
        % move ant
        antCarrying = cell2mat(ants(antId,2));
        loc = cell2mat(ants(antId,1));
        newLoc = [0,0];
        while isequal(newLoc,[0,0])
            stepSize = randi([1,20]);
            %stepSize = 1;
            step = randi([-1*stepSize,1*stepSize],1,2);
            newLoc = (loc + step);
            newLoc = min(max(newLoc,1),gridSize);
            if antMat(newLoc(1),newLoc(2)) ~= 0
                % ant already at location move again
                newLoc = [0,0];
            elseif foodMat(newLoc(1),newLoc(2)) ~= 0
                % location contains food
                if antCarrying ~= 0
                    % ant is already carrying food move again
                    newLoc = [0,0];
                end
            end
        end
        
        % update ant and food location
        ants(antId,1) = {newLoc};
        antMat(loc(1),loc(2)) = 0;
        antMat(newLoc(1),newLoc(2)) = antId;
        if antCarrying ~= 0
            food(antCarrying,2) = {newLoc};
            %foodMat(loc(1),loc(2)) = 0;
            %foodMat(newLoc(1),newLoc(2)) = antCarrying;
        end
        xPos = newLoc(1);
        yPos = newLoc(2);
        
        % find neighboring objects
        neighbors = 0;
        nCount = 0; % neighboring count
        for n = 1:nSize
            tempNeighbors = [xPos yPos+n; xPos+n yPos+n; xPos+n yPos; xPos+n yPos-n; ... 
                             xPos yPos-n; xPos-n yPos-n; xPos-n yPos; xPos-n yPos+n];
            for i = 1:8
                if (tempNeighbors(i,1) > 0 && tempNeighbors(i,1) <= gridSize) && ...
                        (tempNeighbors(i,2) > 0 && tempNeighbors(i,2) <= gridSize)
                    if foodMat(tempNeighbors(i,1),tempNeighbors(i,2)) ~= 0
                        % add foodId to neighbor list
                        nCount = nCount + 1;
                        neighbors(nCount) = foodMat(tempNeighbors(i,1),tempNeighbors(i,2)); 
                    end
                end
            end
        end
        
        if antCarrying ~= 0
            % check if the ant drops the object
            %evalf = 0;
            total = 0;
            if nCount > 0
                for i = 1:length(neighbors)
                    %distance = euclidianDistance(food(neighbors(i),1),food(antCarrying,1));
                    distance = similarity(food(antCarrying,1),food(neighbors(i),1));
                    total = total + distance;
                    %evalf = evalf + (1-(distance/alpha));
                end
            end
            md = total / (((nSize*2)+1)^2 - 1);
            if md > max_d
                max_d = md;
            end
            density = total / (max_d*(((nSize*2)+1)^2 - 1));
            density = max(min(density, 1), 0);
            t = exp(-c * density);
            probability = (1-t)/(1+t);
            pDrop = probability;
            if pDrop >= rand
                % drop object
                foodMat(xPos,yPos) = antCarrying;
                food(antCarrying,2) = {[xPos,yPos]};
                ants(antId,2) = {0};
            end
%             evalf = evalf/(s*s);
%             ppick = (1/(pheromoneMat(xPos,yPos)*evalf))*(kp/(kp+evalf))^2;
%             pdrop = (pheromoneMat(xPos,yPos)*evalf)*(evalf/(kd+evalf))^2;
%             if pdrop > ppick
%                 % drop object
%                 ants(antId,2) = {0};
%                 pheromoneMat(xPos,yPos) = pheromoneMat(xPos,yPos) + pheromoneIncerement*pheromoneMat(xPos,yPos);
%                 if(pheromoneMat(xPos,yPos) > phimax)
%                     pheromoneMat(xPos,yPos) = phimax;
%                 end
%             end
        else
            if foodMat(xPos,yPos) ~= 0
                % Check if the ant picks up the object
                foodId = foodMat(xPos,yPos);
                %evalf = 0;
                total = 0;
                if nCount > 0
                    for i = 1:length(neighbors)
                        %distance = euclidianDistance(food(neighbors(i),1),food(foodId,1));
                        distance = similarity(food(neighbors(i),1),food(foodId,1));
                        total = total + distance;
                        %evalf = evalf + (1-(distance/alpha));
                    end
                end
                md = total / (((nSize*2)+1)^2 - 1);
                if md > max_d
                    max_d = md;
                end
                density = total / (max_d*(((nSize*2)+1)^2 - 1));
                density = max(min(density, 1), 0);
                t = exp(-c * density);
                probability = (1-t)/(1+t);
                pPickUp = 1 - probability;
                if pPickUp >= rand
                    % pick up object
                    ants(antId,2) = {foodId};
                    foodMat(xPos,yPos) = 0;
                end
%                 evalf = evalf/(s*s);
%                 ppick = (1/(pheromoneMat(xPos,yPos)*evalf))*(kp/(kp+evalf))^2;
%                 pdrop = (pheromoneMat(xPos,yPos)*evalf)*(evalf/(kd+evalf))^2;
%                 if pdrop > ppick
%                     % pick up object
%                     ants(antId,2) = {foodId};
%                     pheromoneMat(xPos,yPos) = pheromoneDecrement*pheromoneMat(xPos,yPos);
%                     if(pheromoneMat(xPos,yPos) < phimin)
%                         pheromoneMat(xPos,yPos) = phimin;
%                     end
%                 end
            end
        end
        pause(.01); cla;
        plot(getcol(cell2mat(ants(:,1)),1),getcol(cell2mat(ants(:,1)),2),'r*');
        plot(getcol(cell2mat(food(classA,2)),1),getcol(cell2mat(food(classA,2)),2),'co');
        plot(getcol(cell2mat(food(classB,2)),1),getcol(cell2mat(food(classB,2)),2),'mo');
%         plot(getcol(cell2mat(food(classC,2)),1),getcol(cell2mat(food(classC,2)),2),'yo');
%         plot(getcol(cell2mat(food(classD,2)),1),getcol(cell2mat(food(classD,2)),2),'ko');
    end
%     for x = 1:size(pheromoneMat,1)
%         for y = 1:size(pheromoneMat,2)
%             pheromoneMat(x,y)= pheromoneDecrement*pheromoneMat(x,y);
%             if(pheromoneMat(x,y) < phimin)
%                 pheromoneMat(x,y) = phimin;
%             end
%         end
%     end
end

figure(3);
hold on;
grid off;
axis off;
pause(.01); cla;
plot(getcol(cell2mat(food(classA,2)),1),getcol(cell2mat(food(classA,2)),2),'co');
plot(getcol(cell2mat(food(classB,2)),1),getcol(cell2mat(food(classB,2)),2),'mo');
% plot(getcol(cell2mat(food(classC,2)),1),getcol(cell2mat(food(classC,2)),2),'yo');
% plot(getcol(cell2mat(food(classD,2)),1),getcol(cell2mat(food(classD,2)),2),'ko');




%% Function definitions 

function s = similarity(x,y)
    x = cell2mat(x);
    y = cell2mat(y);
    diff = abs(x - y);
    s = sum(diff.^2);
end

function d = euclidianDistance(x,y)
    x = cell2mat(x);
    y = cell2mat(y);
    d = sqrt(sum((x-y).^2));
end










