% participants = [11:24];
participants = [1:24];
fsrc_dir = 'C:/Users/ramelard/Desktop/DATA COLLECTION - TO TRANSFER TO X/P%0.2u/%s/';
% fsrc_dir =  'X:/data/DATA COLLECTION/complete/whole/P%0.2u/%s/';
fdest_dir = 'X:/data/DATA COLLECTION/complete/P%0.2u/%s/';
tstart = 0;
tduration = 29;
query_user = false;

% % Directory structure for P1-P10
% subdirs = {'1torso_overhead', '2head_bedside', '3head_marked_bedside', ...
%            '4torso_marked_overhead', '5feet_top_overhead', ...
%            '6legs_bottom_overhead', '7legs_bottom_bedside', ...
%            '8legs_bottom_marked_bedside', '9legs_bottom_marked_overhead'};
% fps = 60*ones(9,1);

% % Directory structure for P11-P13
% subdirs = {'1head_bedside', '2torso_overhead', '3roi_overhead', ...
%            '4roi_marked_overhead', '5torso_marked_overhead', ...
%            '6head_marked_bedside', '7feet_top_overhead', ...
%            '8legs_bottom_overhead', '9legs_bottom_bedside', ...
%            '10legs_bottom_marked_bedside', '11legs_bottom_marked_overhead'};
% fps = [60, 60, 200, 200, 60, 60, 60, 60, 60, 60, 60];

% Directory structure for P14-P24
% subdirs = {'1head_bedside', '2torso_overhead', '3roi_overhead', ...
%            '4roi_marked_overhead', '5torso_marked_overhead', ...
%            '6head_marked_bedside', '7feet_top_overhead', ...
%            '8legs_bottom_overhead', '9legs_bottom_bedside', ...
%            '10legs_bottom_marked_bedside'};
% fps = [60, 60, 200, 200, 60, 60, 60, 60, 60, 60];

subdirs = {'torso_overhead', 'torso_overhead'};
fps = [60, 60];

% subdirs_dest = subdirs;
subdirs_dest = {'hand', 'fossa'};

% Get all ROIs ahead of time.
ROIs = cell(numel(participants), numel(subdirs));
for p = 1:numel(participants)
  for d = 1:numel(subdirs)
    src_dir = sprintf(fsrc_dir, participants(p), subdirs{d});
    try
    [~, ~, ~, ~, ~, ~, ROIs{p,d}] = ...
      load_ppgi_images(src_dir, fps(d), 0, 5, Inf, []);
    catch
      ROIs{p,d} = [];
    end
  end
end
      
for p = 1:numel(participants)
  parent_dest_dir = sprintf(fdest_dir, participants(p), '');
  for i = 1:numel(subdirs)
    src_dir = sprintf(fsrc_dir, participants(p), subdirs{i});
    dest_dir = sprintf(fdest_dir, participants(p), subdirs_dest{i});
    
    if exist(dest_dir, 'dir')
      if query_user
        query_str = sprintf('Destination %s directory exists. Overwrite?', dest_dir);
        if strcmpi(questdlg(query_str,'Overwrite','Yes','No','No'), 'no')
          continue;
        else
          rmdir(dest_dir,'s');
        end
      else
        disp(sprintf('Destination %s exists. Skipping.', dest_dir))
        continue;
        % rmdir(dest_dir,'s')
      end
    end
    
    if mkdir(parent_dest_dir, subdirs_dest{i}) == 0
      warning('Could not create %s. Skipping.', dest_dir);
      continue;
    end

    D = dir(src_dir);
    if numel(D) > 1
      try
        move_ppgi_video(src_dir, dest_dir, tstart, tstart+tduration, ...
                        fps(i), ROIs{p,i}, query_user);
      catch
        warning('Participant %u failed.',p)
      end
    else
      disp(sprintf('No files found in %s. Skipping.', src_dir))
    end
  end
end