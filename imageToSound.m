close all; clear; clc;

imgSrc = 'C:\Users\enzoe\Downloads\400px-Necker_cube.svg.png';

img = rgb2gray(imread(imgSrc));
img = imresize(img, [NaN 64]);
[ M, N, ~ ] = size(img);      % pixels
img = (sum(img,'all')/(M*N)) - img;
%imgC = 255 - img;

figure;
image(img);

% https://nl.mathworks.com/matlabcentral/answers/13896-fftshift-of-an-image
Fs_x = 100; % pixels per centimeter
Fs_y = 100;

dx = 1/Fs_x; % centimeters per pixel
dy = 1/Fs_y;

[ M, N, ~ ] = size(img);      % pixels
x = dx*(0:N-1)';               % centimeters
y = dy*(0:M-1)';

dFx = Fs_x/N;              % cycles per centimeter
dFy = Fs_y/M;

Fx = (-Fs_x/2:dFx:Fs_x/2-dFx)';     % cycles per centimeter
Fy = (-Fs_y/2:dFy:Fs_y/2-dFy)';
   
fft2result = fftshift(fft2(img));

figure;
subplot(2,1,1);
plot(Fx,abs(fft2result'));
title('FFT X');

subplot(2,1,2);
plot(Fy,abs(fft2result));
title('FFT Y');