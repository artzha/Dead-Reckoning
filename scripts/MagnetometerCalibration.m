M = csvread("magnetometer.csv");

% Conversion to milligauss
A = M;

% Calculate Offsets
offset_x = (max(A(:,1)) + min(A(:,1)) ) / 2.0;
offset_y = (max(A(:,2)) + min(A(:,2)) ) / 2.0;
offset_z = (max(A(:,3)) + min(A(:,3)) ) / 2.0;

A(:,1) = A(:,1) - offset_x;
A(:,2) = A(:,2) - offset_y;
A(:,3) = A(:,3) - offset_z;


figure(1);
hold on
scatter(A(:,1), A(:,2), 'b');
scatter(A(:,1), A(:,3), 'r');
scatter(A(:,2), A(:,3), 'g');
legend('Raw xy', 'Adjusted xy');
title('Magnetometer Readings of Three Axes');
xlabel('uT');
hold off

q