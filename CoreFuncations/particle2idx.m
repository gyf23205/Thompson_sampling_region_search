function idx = particle2idx(x, origin, res)

idx = ceil(bsxfun(@minus, x, origin).' / res);
