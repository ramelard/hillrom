function [mask_face] = rotate_and_face_detect(im, type)

if nargin < 2
  type = 'auto'
end

im = im(:,:,1);

pix_per_mm = 2;

switch type
  case 'auto'
    mask_face = mask_violajones(im);
  case 'manual'
    mask_face = mask_manual(im);
end
end

function [mask_face] = mask_violajones(im)
  matlabversion = version('-release');
  scale = .2;

  done = false;
  figure;
  while ~done
    if str2num(matlabversion(1:end-1)) > 2014
      FDetect = vision.CascadeObjectDetector('FrontalFaceCART','UseROI',true);
    else
      FDetect = vision.CascadeObjectDetector('FrontalFaceCART');
    end

    S1 = [scale 0 0; 0 scale 0; 0 0 1];
    T1 = maketform('affine',S1);
    [imsmall,xdata1,ydata1] = imtransform(im,T1);
    imshow(imsmall)
    [eyex,eyey] = ginput(2);
    if isempty(eyex)
      mask_face = [];
      return;
    end
    theta = atand(diff(eyey)/diff(eyex));

    R1 = [cosd(theta) -sind(theta) 0; sind(theta) cosd(theta) 0; 0 0 1];
    T2 = maketform('affine',R1);
    [I,xdata2,ydata2] = imtransform(imsmall, T2);

    imshow(I)
  %   rect = getrect;
  %   minsize = [rect(3) rect(4)];
  %   FDetect.MinSize = floor(minsize);
    if str2num(matlabversion(1:end-1)) > 2014
      roi = getrect;
      BB = step(FDetect,I,roi);
    else
  %     FDetect.MinSize = floor(minsize);
      BB = step(FDetect,I);
    end

    % If we didn't find anything, try again.
    if isempty(BB)
      continue;
    end
    BB = BB(1,:);

    imshow(I); hold on
    for i = 1:size(BB,1)
      rectangle('Position',BB(i,:),'LineStyle','-','EdgeColor','r');
    end
    title('Face Detection');
    hold off;

    if strcmpi(questdlg('Use mask?'),'yes')
      done = true;
    end
  end

  pts0 = [BB(1) BB(2) 1; ...
          BB(1)+BB(3) BB(2) 1; ...
          BB(1)+BB(3) BB(2)+BB(4) 1; ...
          BB(1) BB(2)+BB(4) 1]';
  [um,vm] = tforminv(T2, pts0(1,:)+xdata2(1), pts0(2,:));
  [um,vm] = tforminv(T1, um+xdata1(1), vm);

  figure, imshow(im), hold on
  plot([um um(1)], [vm vm(1)], 'r')

  mask_face = roipoly(im, um, vm);
end

function [mask_face] = mask_manual(im)
  figure, imshow(im)
  [eyex,eyey] = ginput(2);
  if isempty(eyex)
    mask_face = [];
    return;
  end
  theta = atand(diff(eyey)/diff(eyex));
  
  R1 = [cosd(theta) -sind(theta) 0; sind(theta) cosd(theta) 0; 0 0 1];
  T1 = maketform('affine',R1);
  [im2,xdata,ydata] = imtransform(im,T1);
  
  imshow(im2)
  [x y] = ginput(4);
  BB = [min(x) min(y) max(x)-min(x) max(y)-min(y)];
  
  pts0 = [BB(1) BB(2) 1; ...
          BB(1)+BB(3) BB(2) 1; ...
          BB(1)+BB(3) BB(2)+BB(4) 1; ...
          BB(1) BB(2)+BB(4) 1]';
  [um,vm] = tforminv(T1, pts0(1,:)+xdata(1), pts0(2,:));

  figure, imshow(im), hold on
  plot([um um(1)], [vm vm(1)], 'r')

  mask_face = roipoly(im, um, vm);
end