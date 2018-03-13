function C = strsplit(str,delimiter)

  num_tokens = numel(strfind(str, delimiter)) + 1;
  C = cell(num_tokens,1);
  remain = str;
  for i = 1:num_tokens
    [C{i}, remain] = strtok(remain, delimiter);
  end

end