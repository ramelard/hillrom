nusers = 5;
nitems = 10;
nfeatures = 10;

% Initial user factor vectors
p = ones(nfeatures, nusers).*0.1;
p = rand(nfeatures, nusers);
% Item factor vectors
q = ones(nfeatures, nitems).*0.1;
q = rand(nfeatures, nitems);

% Known user-item ratings (training set)
r = sparse(nusers, nitems);
% r(1:,:) = randi(6,nitems,1)-1;
% r(2,:) = r(1,:);
% r(2,:) = 5-r(1,:);
% r(2,(r(2,:)==0)) = 1;
r = randi(6,nusers,nitems)-1;

% r = [
%   5	5	-1	2	-1	2	4	-1	5	-1
% 4	5	-1	2	-1	2	4	-1	5	-1
% 1	3	-1	5	-1	4	2	1	2	-1
% ];
r(r<0) = 0;
r=sparse(r);

% Get training set.
[i,j,s] = find(r);
K = [i j s];

% Learning rate
gamma = 0.001;
% Vector size penalization
lambda = 0;

% Stochastic gradient descent
err_t = zeros(1000,1);
for t = 1:numel(err_t)
  e = sparse(size(r));
  for k = 1:size(K,1)
    u = K(k,1);
    i = K(k,2);
    rui = K(k,3);
    qi = q(:,i);
    pu = p(:,u);

    e(u,i) = rui - qi'*pu;
    q(:,i) = qi + gamma*(e(u,i)*pu - lambda*qi);
    p(:,u) = pu + gamma*(e(u,i)*qi - lambda*pu);
  end
  err_t(t) = sum(abs(e(r>0)));
end
figure, plot(err_t)

disp('rui: (0 means no known rating)')
disp(full(r))
disp('q^T*p:')
disp(p'*q)