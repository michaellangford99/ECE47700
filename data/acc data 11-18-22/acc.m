c = csvread("putty.log");
d = c;#c(270:end,1:end);

decimation = 100;

acc_x = d(:,1);
acc_y = d(:,2);
acc_z = d(:,3)-mean(d(:,3));

period = d(1, 4);
Fs = 1/(decimation * period);

L = length(acc_x)

f = Fs*(0:L-1)/L - Fs/2;

Ex = sum(abs(acc_x).^2)
Ey = sum(abs(acc_y).^2)
Ez = sum(abs(acc_z).^2)

X = fftshift(fft(acc_x));
Y = fftshift(fft(acc_y));
Z = fftshift(fft(acc_z));

figure();
plot(f, (abs(X)/max(abs(X))));
figure();
plot(f, (abs(Y)/max(abs(Y))));
figure();
plot(f, (abs(Z)/max(abs(Z))));

figure();
plot((0:L-1)*period*decimation, acc_x);

DC_X = fft(acc_x)(1)
DC_Y = fft(acc_y)(1)
DC_Z = fft(acc_z)(1)

#look at cross correlation between the ffts

rxz = conv(abs(X), flipud(abs(Z)));
figure();
plot(rxz);
