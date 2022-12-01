function box = bounding_box(poly)
% [xmin, ymin, xmax, ymax]

box = [min(poly(:,1)), min(poly(:,2)), max(poly(:,1)), max(poly(:,2))];
