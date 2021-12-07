clear all, close all, clc

%% Load and tweak data
xdata = load('xdata.txt'); % Load the data, we can either store the joint 
                           % angles in a file or pass it directly
col = 1; % So we know which "joint" we are working on
x = xdata(:, col); % xdata contains all the tests results, thus why we only 
                 % use one column at a time
y = x; % Ideally y is the same as x (later we add Gaussian noise)

% Gaussian noise (this needs some tweaking, every time it runs, it gives a 
% different number)
gnoise = y.*randn(size(y));
y = y + 0.2;

plot(x, y, 'o')

n = numel(x); % Returns the number of elements, n, in the array x
[max, min] = maxmin(xdata, col);

%% Slope and intercept  
% (we know the equation, just need to compute each individual component)

sum_x = sum(x);
sum_y = sum(y);
sum_xx = sum(x.*x);
sum_yy = sum(y.*y);
sum_xy = sum(x.*y);
sum_x2 = sum_x*sum_x;
sum_y2 = sum_y*sum_y;

m = (n*sum_xy - sum_x*sum_y)/(n*sum_xx - sum_x2); % Slope
c = (1/n)*(sum_y - m*sum_x); % Intercept

%% Compute the errors in fitting
s = sqrt((sum_yy - sum_y2/n-m*(sum_xy-sum_x*sum_y/n))/(n-2));
e_m = s*sqrt(n*sum_xx - sum_x2); % Error in slope
e_c = s*sqrt(sum_xx/(n*sum_xx - sum_x2)); %Error in intercept

% Correlation Coefficient, tells us how "good" is the fit
R = (sum_xy - sum_x*sum_y/n)/sqrt((sum_xx - sum_x2/n)*(sum_yy - sum_y2/n));

%% Coefficients
xn = linspace(min, max); % Generate linearly spaced vector

yn = m.*xn + c; % Best fit line

e_m
e_c
R

plot(xn, yn, 'LineWidth', 1, 'Color', 'k');
hold on
plot(x, x, 'LineWidth', 5, 'Color', 'r'); % No errors
hold on
plot(x, y, 'o');