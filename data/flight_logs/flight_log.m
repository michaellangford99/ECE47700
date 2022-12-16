c = csvread("log_2022-12-12_17-43-58.txt");


#plot(c(:, 1));
hold on;
plot(c(:, 2));
hold on;
plot(c(:, 3));
#plot(abs(fftshift(fft(c(:, 3)))))