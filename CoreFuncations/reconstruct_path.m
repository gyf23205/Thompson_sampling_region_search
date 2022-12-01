function path = reconstruct_path(next, start, goal)

path = [];
path = iteration(path, next, start, goal);
end

function path = iteration(path, next, start, goal)
    n = next(start, goal);
    if n > 0
        path = [iteration(path, next, start, n) iteration(path, next, n, goal)];
    else
        path = goal;
    end
end
