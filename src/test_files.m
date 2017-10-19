folder = 'home/djole/Downloads/Master/R_&_D/KDL_GIT/Testing_repo/build';
fullFileName = fullfile('poses.txt');
if exist(fullFileName, 'file')
  errorMessage = sprintf('Error: file does not exist:\n%s', fullFileName);
  uiwait(warndlg(errorMessage));
  return;
end
% File exists if you get here.  Now try to actually open it.
  fileID=fopen(fullFileName);
  if fileID == -1
    errorMessage = sprintf('Error: opening file:\n%s', fullFileName);
    uiwait(warndlg(errorMessage));
    return;
  end

A = fscanf(fileID,'%f',[1 inf])
fclose (fileID);
