fs = 100;
t = 0:1/fs:2;
x = sin(2*pi*t*1.5) + sin(2*pi*t*3.5);

N = numel(x);
nvec = 0:N-1;
% nvec = linspace(0.5, 3.5, N);

X = zeros(1,numel(nvec));
% for k = linspace(0.5,3.5,numel(X))*N/fs
for k = 1:numel(X)
  X(k) = sum(x.*exp(-2*pi*1i*(k-1).*nvec./N));
%   X(k) = sum(x.*exp(-2*pi*1i*linspace(0.5, 3.5, N)));
end

figure, plot(abs(X))

%%
fs = 100;
t = 0:1/fs:2;
x = sin(2*pi*t*1.5) + sin(2*pi*t*3.5);
% x = sin(2*pi*t*1.5+pi/4);


t = tppg;
fs = numel(tppg)/(tppg(end)-tppg(1));
x = ppg;

omega = [0.5 : .3 : 4.5];
% omega = linspace(0.5, 3.5, fs/8);
F = [sin(2*pi*t'*omega) cos(2*pi*t'*omega)];
% F = exp(-2*pi*1i*t'*omega);

% A = inv(F'*F)*F'*x';
% A = F\x';
lambda = 0;
% L = ones(numel(t), numel(omega))./numel(t);
L = -1*eye(size(F));
for r = 1:min(size(L))-1
  L(r,r+1) = 1;
end

A = pinv(F'*F+lambda*(L'*L))*F'*x';

figure, plot(omega, sqrt(A(1:numel(omega)).^2+A(numel(omega)+1:end).^2))