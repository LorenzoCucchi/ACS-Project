%{
    Aerospace Control Systems:
    Robust stability and performance analysis of a quadrotor drone.

    config

    Configuration setup for the whole code

    authors: ...

%}

%  config:

run.task0 = true;

run.task1 = true;

run.task2 = true;

run.task3 = true;

run.optimization_innerCycle = false;
run.montecarloSimulation  = false;

dim = 10; % how many values between the boundaries do you want to cycle?

%%% flags for figures:
flag.RelevantImages              = false;
flag.RelevantImagesNO            = false;
flag.PlantAnalysisImages         = false;

flag.task2plot                   = false;

% inner Loop
flag.NominalDesignReqImages_i    = false;
flag.RobustStabilityImages_i     = false;
flag.StructuredHinfImages_i      = false;

% outer Loop
flag.NominalDesignReqImages_o    = false;
flag.RobustStabilityImages_o     = false;
flag.StructuredHinfImages_o      = false;

% montecarlo
flag.MontecarloImages            = false;
warning('off','all')