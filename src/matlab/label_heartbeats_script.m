data_dir = 'C:/Users/ramelard/Desktop/data/experiments/';
users = {'RA','AW','BL','CS','FK'};
tests = {'test1','test2','test5'};

for i = 1:numel(users)
  for j = 1:numel(tests)
    label_heartbeats(sprintf('%s%s/%s/',data_dir,users{i},tests{j}));
    while input('Redo? (y/n) ','s') == 'y'
      label_heartbeats(sprintf('%s%s/%s/',data_dir,users{i},tests{j}));
    end
  end
end