function [tppg, ppg, ppg_tidx] = load_ppg(data_dir, fstart_idx, fstop_idx, fps)

tppg = [];
ppg = [];
if nargout > 2
  ppg_tidx = [];
end

% Try to load ppg data from saved file.
ppg_found = false;
ppg_csv_file = sprintf('%s/tppg_ppg.csv', data_dir);
ppg_mat_file = sprintf('%s/tppg_ppg.mat', data_dir);
if exist(ppg_csv_file, 'file') == 2
  fprintf(1,'Loading data file %s\n', ppg_csv_file)
  M = csvread(ppg_csv_file);
  tppg = M(:,1);
  ppg = M(:,2);
  ppg_found = true;
elseif exist(ppg_mat_file, 'file') == 2
  fprintf(1,'Loading data file %s\n', ppg_mat_file)
  load(ppg_mat_file)
  ppg_found = true;
end

% Normalize loaded PPG
if ppg_found
  tppg = tppg - tppg(1);
  
  if fstop_idx/fps > tppg(end)
    warning('fstop_idx %u beyond last frame %u', fstop_idx, tppg(end)*fps)
    tppg = [];
    ppg = [];
    return;
  end
  
  [~,sidx] = min(abs(tppg-fstart_idx/fps));
  [~,eidx] = min(abs(tppg-fstop_idx/fps));
  tppg = tppg(sidx:eidx);
  ppg = ppg(sidx:eidx);
end

if nargout > 2
  ppg_tidx = [];
  % Try to load ppg data from saved file.
  ppg_tidx_file = sprintf('%s/ppg_tidx.mat', data_dir);
  if exist(ppg_tidx_file, 'file') == 2
    fprintf(1,'Loading data file %s\n', ppg_tidx_file)
    load(ppg_tidx_file)

    sidx = find(ppg_tidx-fstart_idx >= 0, 1, 'First');
    eidx = find(ppg_tidx-fstop_idx > 0, 1, 'First') - 1;
    if isempty(eidx)
      eidx = numel(ppg_tidx);
    end
    ppg_tidx = ppg_tidx(sidx:eidx) - fstart_idx;

    if ppg_tidx(1) == 0
      ppg_tidx(1) = 1;
    end
  end
end

end