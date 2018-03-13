
function output = kalmanexample(x)

for i = 1:size(x,2)
  output = kalmanfilt(x(:,i),3e-5,3e-4);
end

end