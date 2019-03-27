
hr = 67;
rr = 5;

t = 0:1/30:20;
sinx = sin(2*pi*t*hr/60)/2+0.5;

frames_head = rand(600,400,numel(t));
% frames_head = repmat(shiftdim(sinx,-1),[300 400]);
for i = 100:105
  for j = 100:105
    frames_head(i,j,:) = sinx + 0.1*randn(size(sinx));
  end
end


sinx = sin(2*pi*t*rr/60)/2+0.5;

frames_body = rand(600,400,numel(t));
% frames_head = repmat(shiftdim(sinx,-1),[300 400]);
for i = 100:105
  for j = 100:105
    frames_body(i,j,:) = sinx + 0.1*randn(size(sinx));
  end
end
% frames_head = frames_head + 0.1*randn(size(frames_head));

%%

[hr,rr] = extract_vitals_tk1(frames_head, frames_body, ...
  t, 30, 6)