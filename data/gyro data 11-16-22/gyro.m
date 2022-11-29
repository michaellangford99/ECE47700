c = csvread("gyro_data_500dps.csv");
d = c(270:end,1:end);

gyro_rate_x = d(:,1);
gyro_rate_y = d(:,2);
gyro_rate_z = d(:,3);

period = d(1, 10);
Fs = 1/period

L = length(gyro_rate_x)

f = Fs*(0:L-1)/L - Fs/2;

Ex = sum(abs(gyro_rate_x).^2)
Ey = sum(abs(gyro_rate_y).^2)
Ez = sum(abs(gyro_rate_z).^2)

X = fftshift(fft(gyro_rate_x));
Y = fftshift(fft(gyro_rate_y));
Z = fftshift(fft(gyro_rate_z));

#figure();
#hold on;
#plot(f, 20*log10(abs(X)/max(abs(X))));
#plot(f, 20*log10(abs(Y)/max(abs(Y))));
#plot(f, 20*log10(abs(Z)/max(abs(Z))));

DC_X = fft(gyro_rate_x)(1)
DC_Y = fft(gyro_rate_y)(1)
DC_Z = fft(gyro_rate_z)(1)

fs = 100;
t = 0:1/fs:100;
x = [sin(2*pi*7*t), sin(2*pi*12*t), sin(2*pi*2*t)];
specgram(x,64,1000, [],20);

#look at cross correlation between the ffts

#rxz = conv(abs(X), flipud(abs(Z)));
#figure();
#plot(rxz);


