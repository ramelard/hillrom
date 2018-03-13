function save_video(filename, frames, profile, varargin)

if nargin < 3 || isempty(profile)
  writer = VideoWriter(filename);
else
  writer = VideoWriter(filename, profile);
end

for i = 1:2:nargin-3
  set(writer, varargin{i}, varargin{i+1});
end

open(writer);
for i = 1:size(frames,3)
  writeVideo(writer, frames(:,:,i));
end
close(writer);

end