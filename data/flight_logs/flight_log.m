c = csvread("log_2022-12-09_12-17-34.txt");


#plot(c(:, 1));
#hold on;
plot(c(:, 3));
#plot(abs(fftshift(fft(c(:, 3)))))