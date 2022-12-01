function h = communication(h, t)

Nr = length(h.robots);

% PEER-TO-PEER COMMUNICATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~isinf(h.params.comm.Rc_server)
    % Find communication groups
    % if ~isfield(options, 'independent') || (isfield(options, 'independent') && options.independent == false)
        h = group_sensors(h,'Comm');
    % else
    %     h.temp.CommGroups = {};
    % end

    % initialize data structures
    for i = 1:Nr
        r = h.robots(i);

        r.temp.q = zeros(length(r.q), Nr);
        r.temp.q(:,r.id) = r.q;

        h.robots(i) = r;
    end

    % Exchange locations and current measurements
    for k = 1:length(h.temp.CommGroups)
        group = h.temp.CommGroups{k};
        % send pairwise information
        for i = 1:length(group)-1
            for j = i+1:length(group)
                a = group(i); b = group(j);
                ra = h.robots(a); rb = h.robots(b);

                % swap current locations
                ra.temp.q(:,b) = rb.q;
                rb.temp.q(:,a) = ra.q;

                % swap current measurements
                idx = length(ra.comm.in.msg) + 1;
                ra.comm.in.msg(idx) = rb.comm.in.msg(1);
                idx = length(rb.comm.in.msg) + 1;
                rb.comm.in.msg(idx) = ra.comm.in.msg(1);

                h.robots(a) = ra; h.robots(b) = rb;
            end
        end
    end
end

% SERVER COMMUNICATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% send messages to server
s = h.server;
for i = 1:Nr % for each robot
    [d, ~] = pdist2(h.server.access_points', h.robots(i).q', 'euclidean', 'Smallest', 1);
    if d < s.comm.Rc % if can talk to server
        r = h.robots(i);
        while ~isempty(r.comm.out.msg) % send messages and clear buffer
            idx = length(s.comm.in.msg) + 1;
            s.comm.in.msg(idx) = r.comm.out.msg(1);
            r.comm.out.msg(1) = [];
        end
        h.robots(i) = r;
    end
end

% update belief state at server
while ~isempty(s.comm.in.msg)
    s = phd_filter_update(s, s.comm.in.msg(1));
    s.comm.in.msg(1) = [];
end
s.data.lambda(t+1) = s.state.lambda;

% receive belief state from server
for i = 1:Nr % for each robot
    [d, ~] = pdist2(h.server.access_points', h.robots(i).q', 'euclidean', 'Smallest', 1);
    if d < s.comm.Rc % if can talk to server
        h.robots(i).state.lambda = s.state.lambda;
        h.robots(i).state.w = s.state.w*s.state.map;
        h.robots(i).comm.in.msg = struct('q', {}, 'z', {}); % clear buffer
        if h.robots(i).temp.checkin_mode && ~h.robots(i).temp.explore_mode
            h.robots(i).temp.path = [];
        end
%         h.robots(i).data.visited = s.data.visited; % get locations visited by team
        h.robots(i).temp.checkin_mode = false; % reset path to nearest server
    end
end

% s = fit_targets(s, t, h.params.env.targets(1:2,:)); % find targets in server PHD

h.server = s;
