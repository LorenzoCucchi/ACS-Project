%% ATTITUDE CONTROL DESIGN

%%% nominal design requirements - inner loop
set.omega_n_i   = 10;                           % target frequency [rad/s]
set.xi_i        = 0.9;                          % damping factor
set.M_i         = 1.35;
set.A_i         = 1e-3;
set.omega_b_i   = 0.9*set.omega_n_i;
set.alpha_i     = deg2rad(10)/5;
set.omega_tau_i = 16;                           % actuator time constant


% plot related arrays
set.omega_min = 1e-4;                               % [Hz]
set.omega_max = 1e4;                                % [Hz]
set.omega_vec = logspace(log10(set.omega_min),log10(set.omega_max),500);

set.t_min = 0;                                      % [s]
set.t_max = 6;                                      % [s]
set.t_vec = linspace(set.t_min,set.t_max,10^4);

set.u_i = 0*(set.t_vec<=1) + 10*(set.t_vec>1 & set.t_vec<= 3) - 10*(set.t_vec>3 & set.t_vec<= 5) + 0*(set.t_vec>5);
set.u_i = deg2rad(set.u_i);


% hinfstruct options
set.opt = hinfstructOptions('TargetGain',0,'Display','off','RandomStart',50);
% settings.opt = hinfstructOptions('Display','Final','RandomStart',10);


% regulators:
tsk1.R_phi   = regulator_tuner(1,'R_phi','e_\phi','p_0','P');
tsk1.R_p     = regulator_tuner(2,'R_p',{'p_0','p'},{'\delta_{lat}'},'PID');

% summation node:
tsk1.SumInner1 = sumblk('e_\phi = \phi_0 - \phi');

%% NOMINAL DESIGN REQUIREMENTS

s = tf('s');

set.F_2_i = set.omega_n_i^2/(s^2 + 2*s*set.xi_i*set.omega_n_i + set.omega_n_i^2); % target
set.S_2_i = 1 - set.F_2_i; % target

% weight function for p
tsk1.W_p_i = (s/set.M_i + set.omega_b_i)/(s + set.A_i*set.omega_b_i);

% weight function for q
tsk1.W_q_i = set.alpha_i*(s+set.omega_tau_i*1e-3)/(s+set.omega_tau_i);
%tsk1.W_q_i = set.alpha_i*s/s;
if flag.NominalDesignReqImages_i
    figure
    plotoptions = timeoptions;
    plotoptions.YLabel.FontSize = 14;
    plotoptions.XLabel.FontSize = 14;
    plotoptions.Title.FontSize = 15;
    plotoptions.Title.FontWeight = "bold";
    step(set.F_2_i,plotoptions)
    grid on
    
    stepinfo(set.F_2_i)
    
    figure
    bodemag(set.F_2_i)
    hold on
    bodemag(set.S_2_i)
    grid on
    legend('$F\_2$','$S\_2$','interpreter','latex')

    figure
    bodemag(set.F_2_i)
    hold on
    bodemag(set.S_2_i)
    grid on
    bodemag(1/tsk1.W_p_i)
    legend('$F\_{2i}$','$S\_{2i}$','$1/W\_{pi}$','interpreter','latex','location','southeast','FontSize',13)

    figure
    plotoptions = bodeoptions;
    plotoptions.YLabel.FontSize = 14;
    plotoptions.XLabel.FontSize = 14;
    plotoptions.TickLabel.FontSize = 10;
    plotoptions.Title.FontSize = 15;
    plotoptions.Title.FontWeight = "bold";
    bodemag(1/tsk1.W_q_i,plotoptions)
    grid on

    figure 
    yyaxis left
    plot(set.t_vec,set.u_i)
    ylabel("$\varphi_{0}$","FontSize",14)
    hold on
    yyaxis right
    yline([-5 5],"--","LineWidth",2)
    ylabel("$\delta_{lat}$","FontSize",14)
    xlabel("Time [s]")
    title("Doublet change in $\varphi_0$")
    grid on
end

%% Structured Hinf synthesis - Inner loop

tsk1.W_p_i.u = 'e_\phi';
tsk1.W_p_i.y = 'z_1';

tsk1.W_q_i.u = '\delta_{lat}';
tsk1.W_q_i.y = 'z_2';
tsk1.CL0_i = connect(tsk1.R_phi, tsk1.R_p, set.sys_p_phi_nom, set.sys_delta_p_nom, tsk1.SumInner1,...
    tsk1.W_p_i, tsk1.W_q_i,'\phi_0',{'z_1','z_2'},{'\delta_{lat}','e_\phi','\phi'});


% CL_i = inner closed loop
[tsk1.CL_i,~,~] = hinfstruct(tsk1.CL0_i,set.opt); % as you can see the output of this hinfstruct gives a peak gain = 0.798 which is not optimal, we want it to be the closest possible to 1 (while being smaller than 1)

showTunable(tsk1.CL_i);

tsk1.F_i = getIOTransfer(tsk1.CL_i,'\phi_0','\phi');

tsk1.S_i = getIOTransfer(tsk1.CL_i,'\phi_0','e_\phi');
tsk1.Q_i = getIOTransfer(tsk1.CL_i,'\phi_0','\delta_{lat}');
tsk1.L_i = (1 - tsk1.S_i)/tsk1.S_i;

%%
if flag.StructuredHinfImages_i
    figure
        bodemag(tsk1.F_i)
        grid on
   

    figure
        step(tsk1.F_i)
        hold on
        step(set.F_2_i)
        grid on
        legend('hinfstruct','second order', 'location','southeast')
        
    stepinfo(tsk1.F_i)

    figure
        bodemag(tsk1.Q_i)
        grid on
        hold on
        bodemag(1/tsk1.W_q_i)
        legend('$Q$','$1/W_q$','interpreter','latex', 'location','southeast')

    % figure
    %     margin(tsk1.L_i)
    %     grid on

    figure
    title("Doublet change in $\varphi_0$")
    yyaxis left
    plot(set.t_vec,set.u_i)
    ylabel("$\varphi_{0}$","FontSize",18)
    yticks([-0.2 -0.15 -0.1 -0.05 0 0.05 0.1 0.15 0.2])
    hold on
    yyaxis right
    ylim([-5 5])
    xlim([0 6])
    yticks([-5 -4 -3 -2 -1 0 1 2 3 4 5])
    [y_out, time] = lsim(tsk1.Q_i,set.u_i,set.t_vec);
    plot(time,y_out)
    ylabel("$\delta_{lat}$","FontSize",18)
    xlabel("Time [s]")
    hold on
    grid on
end


if flag.RelevantImages
    figure
        bodemag(tsk1.S_i)
        grid on
        hold on
        bodemag(1/tsk1.W_p_i)
        legend('$S$','$1/W_p$','interpreter','latex', 'location','southeast')
end

%% Robust Stability Analysis

tsk1r.sys_delta_p_array = usample(set.sys_delta_p,100);
[~,Info] = ucover(tsk1r.sys_delta_p_array,set.sys_delta_p_nom,3);

tsk1r.W_delta_p = Info.W1;

% fixed parameters regulators
tsk1r.R_phi = regulator_tuner(0,tsk1.CL_i.Blocks.R_phi,'e_\phi','p_0');
tsk1r.R_p   = regulator_tuner(0,tsk1.CL_i.Blocks.R_p,{'p_0','p'},'\delta_{lat}');

% inner summation block
tsk1r.SumInner1 = sumblk('e_\phi = - \phi');

% weight functions
tsk1r.W_delta_p.u = '\delta_{lat}';
tsk1r.W_delta_p.y = 'z_a';

% uili = uncertain inner loop input
tsk1r.SumInner2 = sumblk('uili = \delta_{lat} + w_a');

tsk1r.sys_delta_p_nom = set.sys_delta_p_nom;
tsk1r.sys_delta_p_nom.u = 'uili';

tsk1r.CL_a = connect(tsk1r.R_phi, tsk1r.R_p, set.sys_p_phi_nom, tsk1r.sys_delta_p_nom, tsk1r.SumInner1,...
    tsk1r.SumInner2, tsk1r.W_delta_p,'w_a','z_a',{'\delta_{lat}','\phi'});

tsk1r.M_a = getIOTransfer(tsk1r.CL_a,'w_a','z_a');

% M_a_ frequency response
tsk1r.M_a_fr= bode(tsk1r.M_a,set.omega_vec);

% Robust performance
tsk1.FW = squeeze(bode(tsk1.F_i*tsk1r.W_delta_p,set.omega_vec));
tsk1.SW = squeeze(bode(tsk1.S_i*tsk1.W_p_i,set.omega_vec));

figure
semilogx(set.omega_vec, (20*log10(tsk1.FW)+20*log10(tsk1.SW)));
hold on
semilogx(set.omega_vec,20*log10(set.omega_vec./set.omega_vec));
grid on 
title('Robust Performance','interpreter','tex','FontWeight','bold')
ylabel('Magnitude (dB)')
xlabel('Frequency (rad/s)')
legend('$|WpS|+|WF|$', '$1$', 'interpreter', 'latex', 'location', 'best')


if flag.RobustStabilityImages_i
    figure
        bodemag(tsk1r.W_delta_p,{set.omega_min,set.omega_max},'r')
        hold on
        %bodemag((tsk1r.sys_delta_p_nom-tsk1r.sys_delta_p_array)/tsk1r.sys_delta_p_nom,{set.omega_min,set.omega_max})
        grid on

    figure % per la presentazione serve
        semilogx(set.omega_vec, 20*log10(abs(squeeze(tsk1r.M_a_fr))))
        hold on
        semilogx(set.omega_vec, 20*log10(set.omega_vec./set.omega_vec))
        grid on
        title('Robust Stability','interpreter','tex','FontWeight','bold')
        ylabel('Magnitude (dB)')
        xlabel('Frequency (rad/s)')
        legend('$|M_a|$','$1$','interpreter','latex', 'location','southeast')
end

%% Inner Loop Optimization

% fixed optimization parameters:
set.M_i = 1.35;
% set.A_i = 1e-3;

% optimization parameters to cycle:
tsk1o.alpha_i_vec = set.alpha_i * linspace(1,3,dim);

tsk1o.omega_b_i_min = set.omega_n_i/sqrt(2) * sqrt((1-1/set.M_i^2)/(1-set.A_i^2)); % if M and A are fixed there is only one value for omega_b_min that does not 
tsk1o.omega_b_i_max = 3 * set.omega_n_i; % non avendo requisiti la poniamo un multiplo della minima

tsk1o.omega_b_i_vec = linspace(tsk1o.omega_b_i_min,tsk1o.omega_b_i_max,dim);

% Optimal parameters (optimization cycle)

if run.optimization_innerCycle
    [tsk1o.omega_b_i_opt,tsk1o.alpha_i_opt,tsk1o.optimization_parameter_i] =...
        innerLoopOptimization(alpha_i_vec,tsk1o.omega_b_i_vec,dim,set);
    save_struct_i = struct('alpha_i_opt',tsk1o.alpha_i_opt,'omega_b_i_opt',tsk1o.omega_b_i_opt,'N_iter',dim,'optimParam_i',tsk1o.optimization_parameter_i.best_value);
    save("M_"+num2str(set.M_i*100)+"innerLoopOpt_DIM_"+num2str(dim),"save_struct_i");
else
    load("innerLoopOpt_DIM_100.mat");
    tsk1o.omega_b_i_opt = save_struct_i.omega_b_i_opt;
    tsk1o.alpha_i_opt = save_struct_i.alpha_i_opt;
    tsk1o.optimization_parameter_i = save_struct_i.optimParam_i;
end

tsk1o.W_p_i = (s/set.M_i + tsk1o.omega_b_i_opt)/(s + set.A_i*tsk1o.omega_b_i_opt);
tsk1o.W_q_i = tsk1o.alpha_i_opt*(s+set.omega_tau_i*1e-3)/(s+set.omega_tau_i);

tsk1o.R_phi = regulator_tuner(1,'R_phi_opt','e_\phi','p_0','P');
tsk1o.R_p   = regulator_tuner(2,'R_p_opt',{'p_0','p'},{'\delta_{lat}'},'PID');

tsk1o.SumInner1 = sumblk('e_\phi = \phi_0 - \phi');

% Structured Hinf synthesis

tsk1o.W_p_i.u = 'e_\phi';
tsk1o.W_p_i.y = 'z_1';

tsk1o.W_q_i.u = '\delta_{lat}';
tsk1o.W_q_i.y = 'z_2';

tsk1o.sys_delta_p_nom = tsk1r.sys_delta_p_nom;
tsk1o.sys_delta_p_nom.u = '\delta_{lat}';

% Optimal (from our code) augmented plant for Nominal Performance and Control Effort Moderation
tsk1o.CL0_i = connect(tsk1o.R_phi, tsk1o.R_p, set.sys_p_phi_nom, tsk1o.sys_delta_p_nom, tsk1o.SumInner1, tsk1o.W_p_i, tsk1o.W_q_i,...
    '\phi_0',{'z_1','z_2'},{'\delta_{lat}','e_\phi','\phi'});


[tsk1o.CL_i,tsk1o.gamma_i,info_i] = hinfstruct(tsk1o.CL0_i,set.opt); % now from the output you can see this is way more optimal than the initial guess, with a peak gain of 0.9999

tsk1o.F_i = getIOTransfer(tsk1o.CL_i,'\phi_0','\phi');
tsk1o.S_i = getIOTransfer(tsk1o.CL_i,'\phi_0','e_\phi');
tsk1o.Q_i = getIOTransfer(tsk1o.CL_i,'\phi_0','\delta_{lat}');

% F_2_i = settings.omega_n_i^2/(s^2 + 2*s*settings.xi*settings.omega_n_i + settings.omega_n_i^2);
% S_2_i = 1-F_2_i;

% figures %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag.RelevantImagesNO
    figure
        bodemag(tsk1o.S_i)
        grid on
        hold on
        bodemag(set.S_2_i)
        bodemag(1/tsk1o.W_p_i)
        legend('$S$', '$S_{target}$', '$\frac{1}{W_p}$','interpreter','latex', 'location','southeast')
end

if flag.RelevantImages
    figure
        bodemag(tsk1o.Q_i)
        grid on
        hold on
        bodemag(1/tsk1o.W_q_i)
        legend('$Q$','$1/W\_q$','interpreter','latex', 'location','southeast')
end
tsk1o.sys_delta_p_array = usample(set.sys_delta_p,100);
[~,Info] = ucover(tsk1o.sys_delta_p_array,tsk1o.sys_delta_p_nom,3);

tsk1o.W_delta_p = Info.W1;

% if flag.RelevantImagesNO
%     figure
%         bodemag(tsk1o.W_delta_p,{set.omega_min,set.omega_max},'r')
%         hold on
%         bodemag((tsk1o.sys_delta_p_nom-tsk1o.sys_delta_p_array)/tsk1o.sys_delta_p_nom,{set.omega_min,set.omega_max})
%         grid on
% end

if flag.RelevantImagesNO
    [mag,~,wout] = bode(tsk1o.W_delta_p,{set.omega_min,set.omega_max});
    Mag=20*log10(mag(:));
    figure
        loglog(wout,Mag,'r','LineWidth',2)
        hold on
        bodemag((tsk1o.sys_delta_p_nom-tsk1o.sys_delta_p_array)/tsk1o.sys_delta_p_nom,{set.omega_min,set.omega_max})
        grid on
        title('Weight for robust stability and robust performance')
        ylim([-65,-20])
end

% Controllori con parametri fissati
tsk1of.R_phi = regulator_tuner(0,tsk1o.CL_i.Blocks.R_phi_opt,'e_\phi','p_0');
tsk1of.R_p   = regulator_tuner(0,tsk1o.CL_i.Blocks.R_p_opt,{'p_0','p'},'\delta_{lat}');
tsk1of.SumInner1 = sumblk('e_\phi = - \phi');

tsk1of.W_delta_p = tsk1o.W_delta_p;
tsk1of.W_delta_p.u = '\delta_{lat}';
tsk1of.W_delta_p.y = 'z_a';

tsk1of.SumInner2 = sumblk('uili = \delta_{lat} + w_a');

tsk1of.sys_delta_p_nom = tsk1o.sys_delta_p_nom;
tsk1of.sys_delta_p_nom.u = 'uili';

%
tsk1of.CL_i = connect(tsk1of.R_phi, tsk1of.R_p, set.sys_p_phi_nom, tsk1of.sys_delta_p_nom, tsk1of.SumInner1,...
    tsk1of.SumInner2, tsk1of.W_delta_p, 'w_a','z_a',{'\delta_{lat}','\phi'});

M_SISO = getIOTransfer(tsk1of.CL_i,'w_a','z_a');
M_SISO_fr= bode(M_SISO,set.omega_vec); % frequency response

if flag.RelevantImages
    figure('Prova')
        semilogx(set.omega_vec, 20*log10(abs(squeeze(tsk1r.M_a_fr))))
        hold on
        semilogx(set.omega_vec, 20*log10(set.omega_vec./set.omega_vec))
        grid on
        legend('$|M_a|$','$1$','interpreter','latex', 'location','southeast')
        save_fig('Prova','Prova');
end
