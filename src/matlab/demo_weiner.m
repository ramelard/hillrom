% im, imblurred
im = im2double(imread('cameraman.tif'));

nmean = 0;
nvar = 0.0001;
noise = sqrt(nvar)*randn(size(im));

LEN = 21;
THETA = 11;
PSF = fspecial('motion', LEN, THETA);
imblurred = imfilter(im, PSF, 'conv', 'circular');
% imblurred = imnoise(imblurred, 'gaussian',nmean,nvar); 
imblurred = imblurred + noise;

y = edgetaper(imblurred,PSF);
y = imblurred;
x = im;
h = PSF;

% H = fft2(h);
H = psf2otf(PSF, size(y));
% S = fft2(x);

% power-to-signal ratio (inverse SNR)
N = nvar / var(im(:));
% N = sum(noise(:).^2)/sum(im(:).^2);
% Mean PSD of x
S = 1;
% S = abs(fft2(x)).^2;
% S = S./sum(S(:));

G = (conj(H).*S)./((abs(H).^2).*S + N);

Xhat = G.*fft2(y);
xhat = real(ifft2(Xhat));

figure, subplot(121), imshow(y), subplot(122), imshow(xhat)

%%
I = checkerboard(8);
noise = 0.1*randn(size(I));
PSF = fspecial('motion',21,11);
Blurred = imfilter(I,PSF,'circular');
BlurredNoisy = im2uint8(Blurred + noise);

NSR = sum(noise(:).^2)/sum(I(:).^2);% noise-to-power ratio
       
NP = abs(fftn(noise)).^2;% noise power
NPOW = sum(NP(:))/prod(size(noise));
% 2D Auto-correlation based on Wiener-Khinchin theorem:
% Autocorr is simply the Fourier transform of absolute square of F coefs.
% (I think ifft is used here to normalize)
NCORR = fftshift(real(ifftn(NP)));% noise autocorrelation 

IP = abs(fftn(I)).^2;% original image power
IPOW = sum(IP(:))/prod(size(I));
ICORR = fftshift(real(ifftn(IP)));% image autocorrelation 
% ICORR = xcorr2(I,I);
ICORR1 = ICORR(:,ceil(size(I,1)/2));

NSR = NPOW/IPOW;
figure
subplot(221);imshow(BlurredNoisy,[]);
title('A = Blurred and Noisy');
subplot(222);imshow(deconvwnr(BlurredNoisy,PSF,NSR),[]);
title('deconvwnr(A,PSF,NSR)');
subplot(223);imshow(deconvwnr(BlurredNoisy,PSF,NCORR,ICORR),[]);
title('deconvwnr(A,PSF,NCORR,ICORR)');
subplot(224);imshow(deconvwnr(BlurredNoisy,PSF,NPOW,ICORR1),[]);
title('deconvwnr(A,PSF,NPOW,ICORR_1_D)');