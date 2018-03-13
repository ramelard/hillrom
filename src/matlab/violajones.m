data_dir = 'C:/Users/ramelard/Desktop/data/headtrack/';

faceDetector = vision.CascadeObjectDetector;
% faceDetector.ClassificationModel = 'FrontalFaceLBP';
% faceDetector.ClassificationModel = 'FrontalFaceCART';
faceDetector.ClassificationModel = 'ProfileFace';
% faceDetector.ClassificationModel = 'Mouth';

nframes = 200;
frames = [];
h = waitbar(0,'');
for i = 1:nframes
  waitbar(i/nframes, h)
  [I, ~] = load_ppgi_images(data_dir, 80, i-1, i-1, Inf);
  I = imresize(I, [250 250]);
  bboxes = step(faceDetector, I);
  
  if isempty(frames)
    frames = zeros(250,250,nframes);
  end
  
  frames(:,:,i) = rgb2gray(insertObjectAnnotation(I, 'rectangle', bboxes, faceDetector.ClassificationModel));
end
close(h)

implay(frames,80)




%%
im = [];
nframes = 396;
h = waitbar(0,'');
for i = 1:nframes
  waitbar(i/nframes, h)
  [I, ~] = load_ppgi_images(data_dir, 80, i-1, i-1, Inf);
  I = imresize(I, [250 250]);
  
  if isempty(im)
    im = zeros(250,250,nframes);
  end
  
  im(:,:,i) = I;
end
close(h)

%%
faceDetector = vision.CascadeObjectDetector;
% faceDetector.ClassificationModel = 'FrontalFaceLBP';
% faceDetector.ClassificationModel = 'FrontalFaceCART';
faceDetector.ClassificationModel = 'ProfileFace';
% faceDetector.ClassificationModel = 'Mouth';

nframes = size(im,3);
frames = [];
h = waitbar(0,'');
for i = 1:nframes
  waitbar(i/nframes, h)
  I = im(:,:,i);
  bboxes = step(faceDetector, I);
  
  if isempty(frames)
    frames = zeros(250,250,nframes);
  end
  
  frames(:,:,i) = rgb2gray(insertObjectAnnotation(I, 'rectangle', bboxes, faceDetector.ClassificationModel));
end
close(h)

implay(frames,80)