%{
    Aerospace Control Systems:
    Robust stability and performance analysis of a quadrotor drone.

    Main code

    Called Scripts: task0, task1, task2, task3, task4
%% 

    authors: 

%}

clear
close all
clc

%% PATH
%filePath = fileparts(mfilename('fullpath'));
%currentPath = pwd;

%if not(strcmp(filePath, currentPath))
%    cd (filePath);
%    currentPath = filePath;
%end

%addpath(genpath(currentPath));
addpath('./src');

%%  GRAPHICS
matlab_graphics

%% CONFIG
config

%% TASK 0
if (run.task0)
    task0
end


%% TASK 1
if (run.task1)
    task1
end


%% TASK 2

if (run.task2)
    task2
end


%% TASK 3 Observer

if (run.task3)
    task3
end

