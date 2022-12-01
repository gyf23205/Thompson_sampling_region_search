function h = group_sensors(h, Grid)
% Groups sensors into dependent clusters based on their relative locations

R = ones(1, length(h.robots));
if strcmpi(Grid, 'Sense')
    for i = 1 : length(h.robots)
        R(i) = min(h.robots(i).sensor.Rs, h.robots(i).comm.Rc/2); % radius for finding neighbors
    end
elseif strcmpi(Grid,'Comm')
    for i = 1 : length(h.robots)
        R(i) = h.robots(i).comm.Rc/2;
    end
else
    error('Group type not defined')
end

% create adjacency matrix
d = pdist([h.robots.q]');
d = squareform(d);
D = bsxfun(@plus,R',R);
A = d<D;

% find connected components and assign groups
[p,~,r,~] = dmperm(A);
groups = zeros(1, length(h.robots));
for i = 1:length(r)-1
    groups(p(r(i):r(i+1)-1)) = i;
end

if strcmpi(Grid, 'Sense')
    for i = 1:length(h.robots)
        h.robots(i).temp.Group = find(groups == groups(i));
    end
    h.temp.Coalitions = cell(1,max(groups));
    for i = 1:max(groups)
        h.temp.Coalitions{i} = find(groups == i);
    end
    
elseif strcmpi(Grid,'Comm')
    h.temp.CommGroups = cell(1,length(r)-1);
    count = 1;
    for i = 1:length(r)-1
        if sum(groups==i) > 1 % only count groups that have multiple members
            h.temp.CommGroups{count} = find(groups==i);
            count = count+1;
        end
    end
end
