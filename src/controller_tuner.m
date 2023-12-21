function [R_p, R_phi, CL] = controller_tuner(R_p, R_phi, W_p, W_q, model, varargin)

    Sum = sumblk('e = \phi^0 - \phi');

    if nargin < 6

        plant = connect(R_p, R_phi, tf(model.nominal), Sum, W_p, W_q, ...
            '\phi^0', {'z_1', 'z_2'}, {'\delta', 'e', '\phi'});

    else
        W = varargin{1};
        K = varargin{2};
        actuator = varargin{3};

        [R_p, R_phi] = controller_creator(); 

        plant = connect(R_p, R_phi, tf(model.nominal), Sum, K, actuator, W_p, W_q, W, ...
            '\phi^0', {'z1', 'z2', 'z3'}, {'\delta', 'e', '\phi'});
    end

    opt = hinfstructOptions('Display', 'off', 'RandomStart', 10);
    [CL, gamma, INFO] = hinfstruct(plant, opt);

    for i = 1:length(INFO)

        if INFO(i).Objective == gamma
            R_phi.Kp.Value = INFO(i).TunedBlocks.R_phi.Kp.Value;
            R_p.Kp.Value = INFO(i).TunedBlocks.R_p.Kp.Value;
            R_p.Ki.Value = INFO(i).TunedBlocks.R_p.Ki.Value;
            R_p.Kd.Value = INFO(i).TunedBlocks.R_p.Kd.Value;
        end

    end
end