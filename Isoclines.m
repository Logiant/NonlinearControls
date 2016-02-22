clear; close all; clc;

alpha = 1;
x1 = 0:0.5:2*pi;
x2 = -sin(x1)./alpha;

quiver(x1, x2, ones(length(x1))*alpha, ones(length(x1))*1) 