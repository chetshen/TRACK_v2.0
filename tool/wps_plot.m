sf=10000;
sig=F(:,1);
mother = 'MORLET';
dt = 1/sf;
sst = sig;%(acc - mean(acc))/std(acc) ;
dj = 0.125; %0.125  % resolution
s0 = 2*dt;
j1 = 14/dj;    % change low frequency bound of the plot
%mother = 'PAUL'; 'Morlet'; 'DOG'
[wave,period,scale,coi] = wavelet(sst,dt,1,dj,s0,j1,mother);
% power2 = (abs(wave)).^2 ;
power = wave.* conj(wave);
%phase=atan(imag(wave)./real(wave));
%phase=asin(imag(wave)./sqrt(imag(wave).^2+real(wave).^2));
freq=1./period;


[xx,yy]=meshgrid(X_w,freq);
figure
mesh(xx,yy,power);
view(0,90);
% wp_fe_500_oder5=sum(power,2)./1701;
% wp_max=max(wp);
% wp_norm=wp/wp_max;

