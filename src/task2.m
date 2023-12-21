%% ATTITUDE CONTROL ROBUSTNESS ANALYSIS

tsk2.SumInner1 = sumblk('e_\phi = \phi_0 - \phi');

tsk2.sys_p_phi_nom = set.sys_p_phi_nom;
tsk2.sys_p_phi_nom.u = 'p';
tsk2.sys_p_phi = set.sys_p_phi;
tsk2.sys_p_phi.u = 'p';
tsk2.sys_delta_p_nom = set.sys_delta_p_nom;
tsk2.sys_delta_p_nom.u = '\delta_{act}';
tsk2.sys_delta_p = set.sys_delta_p;
tsk2.sys_delta_p.u = '\delta_{act}';
tsk2.sys_d_db_nom = set.sys_d_db_nom;
tsk2.sys_d_db_nom.u = '\delta_{lat}';

tsk2.R_phi = tsk1r.R_phi;
tsk2.R_p = tsk1r.R_p;
tsk2.W_p_i = tsk1.W_p_i;
tsk2.W_q_i = tsk1.W_q_i;

tsk2.CL2_i = connect(tsk2.R_phi, tsk2.R_p, tsk2.sys_d_db_nom, set.sys_db_da, tsk2.sys_delta_p_nom,...
    tsk2.sys_p_phi_nom, tsk2.SumInner1, tsk2.W_p_i, tsk2.W_q_i, '\phi_0','\phi',{'\delta_{lat}','e_\phi'});
showTunable(tsk2.CL2_i)

tsk2.F2_i = getIOTransfer(tsk2.CL2_i,'\phi_0','\phi');
tsk2.S2_i = getIOTransfer(tsk2.CL2_i,'\phi_0','e_\phi');
tsk2.Q2_i = getIOTransfer(tsk2.CL2_i,'\phi_0','\delta_{lat}');
tsk2.L2_i = (1-tsk2.S2_i)/(tsk2.S2_i);

%%
% figure
% bodemag(W_delta_p, {omega_min, omega_max},'r')
% hold on 
% bodemag((rob_plant.sys_d_p_nom - rob_plant.array)/rob_plant.sys_d_p_nom, {omega_min,omega_max})
% grid on 
% 
% figure 
% semilogx(set.omega_vec, 20*log10(abs(squeeze(M_a_fr))))
% hold on 
% semilogx(set.omega_vec, 20*log10(set.omega_vec./set.omega_vec))
% grid on 
% legend('$|M_a|$', '$1$', 'interpreter', 'latex', 'location', 'southeast')
%if flag.task2plot
    figure()
    hold on 
    lsim(tsk2.F2_i,set.u_i,set.t_vec,'b');
    
    figure
    step(tsk2.F2_i)
    hold on
    step(tsk1o.F_i)
    step(set.F_2_i)
    legend('Complete System','Optim Simp','Target')
    
    stepinfo(tsk2.F2_i)
    figure
    bodemag(tsk2.Q2_i)
    grid on
    hold on
    bodemag(1/tsk2.W_q_i)
    legend('$Q$','$1/W_q$','interpreter','latex')
    
    figure
    margin(tsk2.L2_i)
    
    figure
    lsim(tsk2.Q2_i,set.u_i,set.t_vec);
    grid on
    
    figure
    bodemag(tsk2.S2_i)
    grid on
    hold on
    bodemag(1/tsk2.W_p_i)
    legend('$S$','$1/W_p$','interpreter','latex')
%end



%% Hinf for complete system

tsk2c.SumInner1 = tsk2.SumInner1;

tsk2c.R_phi = regulator_tuner(1,'R_phi','e_\phi','p_0','P');
%% 
tsk2c.R_p = regulator_tuner(2, 'R_p', {'p_0','p'},{'\delta_{lat}'},'PID');

tsk2c.W_p_i = tsk2.W_p_i;
%tsk2c.W_p_i = (s/set.M_i + 12.0)/(s + set.A_i*12.0);
tsk2c.W_p_i.u = 'e_\phi';
tsk2c.W_p_i.y = 'z_1';

tsk2c.W_q_i = tsk2.W_q_i;

tsk2c.CL2_i = connect(tsk2c.R_phi, tsk2c.R_p, tsk2.sys_d_db_nom, set.sys_db_da, tsk2.sys_delta_p_nom, ...
    tsk2.sys_p_phi_nom, tsk2c.SumInner1, tsk2c.W_p_i, tsk2c.W_q_i, '\phi_0',{'z_1','z_2'},{'\delta_{lat}','e_\phi','\phi'});


[tsk2cf.CL3_i,~,~] = hinfstruct(tsk2c.CL2_i,set.opt);
showTunable(tsk2cf.CL3_i)

tsk2cf.R_phi = regulator_tuner(0,tsk2cf.CL3_i.Blocks.R_phi,'e_\phi','p_0');
tsk2cf.R_p = regulator_tuner(0,tsk2cf.CL3_i.Blocks.R_p, {'p_0','p'},'\delta_{lat}');

tsk2cf.SumInner1 = tsk2c.SumInner1;

rob_plant.nominal = connect(tsk2cf.R_phi, tsk2cf.R_p, tsk2.sys_d_db_nom, set.sys_db_da, tsk2.sys_delta_p_nom, tsk2.sys_p_phi_nom, tsk2cf.SumInner1,'\phi_0','\phi');

rob_plant.uncertain = connect(tsk2cf.R_phi, tsk2cf.R_p, set.sys_d_db, set.sys_db_da, tsk2.sys_delta_p, tsk2.sys_p_phi, tsk2cf.SumInner1, '\phi_0','\phi');

rob_plant.sys_d_p = connect(set.sys_d_db, set.sys_db_da, tsk2.sys_delta_p, '\delta_{lat}', 'p');

rob_plant.sys_d_p_nom = connect(tsk2.sys_d_db_nom, set.sys_db_da, tsk2.sys_delta_p_nom,'\delta_{lat}','p');

rob_plant.array = usample(rob_plant.sys_d_p,100);
[~,Info] = ucover(rob_plant.array, rob_plant.sys_d_p_nom, 3);
tsk2cf.W_delta_p = Info.W1;


tsk2cf.SumInner1 = sumblk('e_\phi = -\phi');
tsk2cf.W_delta_p.u = '\delta_{lat}';
tsk2cf.W_delta_p.y = 'z_a';

tsk2cf.SumInner2 = sumblk('uili = \delta_{lat} + w_a');

tsk2cf.sys_d_db_nom = tsk2.sys_d_db_nom;
tsk2cf.sys_d_db_nom.u = 'uili';

tsk2cf.CL_a = connect(tsk2cf.R_phi, tsk2cf.R_p, tsk2cf.sys_d_db_nom, set.sys_db_da, tsk2.sys_delta_p_nom, ...
    tsk2.sys_p_phi_nom, tsk2cf.SumInner1, tsk2cf.SumInner2, tsk2cf.W_delta_p, 'w_a', 'z_a', {'\delta_{lat}','\phi'});

tsk2cf.M_a = getIOTransfer(tsk2cf.CL_a,'w_a','z_a');


tsk2cf.M_a_fr = bode(tsk2cf.M_a, set.omega_vec);

tsk2cf.F3_i = getIOTransfer(tsk2cf.CL3_i,'\phi_0','\phi');
tsk2cf.S3_i = getIOTransfer(tsk2cf.CL3_i,'\phi_0','e_\phi');
tsk2cf.Q3_i = getIOTransfer(tsk2cf.CL3_i,'\phi_0','\delta_{lat}');
tsk2cf.L3_i = (1-tsk2cf.S3_i)/(tsk2cf.S3_i);

% Robust performance
tsk2cf.FW = squeeze(bode(tsk2cf.F3_i*tsk2cf.W_delta_p,set.omega_vec));
tsk2cf.SW = squeeze(bode(tsk2cf.S3_i*tsk2c.W_p_i,set.omega_vec));

figure
semilogx(set.omega_vec, (20*log10(tsk2cf.FW)+20*log10(tsk2cf.SW)));
hold on
semilogx(set.omega_vec,set.omega_vec./set.omega_vec);
grid on 
title('Robust Performance','FontWeight','bold')
ylabel('Magnitude (dB)')
xlabel('Frequency (rad/s)')
legend('$|WpS|+|WF|$', '$1$', 'interpreter', 'latex', 'location', 'best')

%%
if flag.task2plot
    figure
    bodemag(tsk2cf.W_delta_p, {set.omega_min, set.omega_max},'r')
    hold on 
    bodemag((rob_plant.sys_d_p_nom - rob_plant.array)/rob_plant.sys_d_p_nom, {set.omega_min,set.omega_max})
    grid on 
    
    figure 
    semilogx(set.omega_vec, 20*log10(abs(squeeze(tsk2cf.M_a_fr))))
    hold on 
    semilogx(set.omega_vec, 20*log10(set.omega_vec./set.omega_vec))
    grid on 
    title('Robust Stability','interpreter','tex','FontWeight','bold')
    ylabel('Magnitude (dB)')
    xlabel('Frequency (rad/s)')
    legend('$|M_a|$', '$1$', 'interpreter', 'latex', 'location', 'southeast')
    
    
    
    figure
    step(tsk2cf.F3_i)
    hold on
    step(set.F_2_i)
    legend('Complete System','Target')
    
    stepinfo(tsk2cf.F3_i)
    figure
    bodemag(tsk2cf.Q3_i)
    grid on
    hold on
    bodemag(1/tsk2.W_q_i)
    legend('$Q$','$1/W_q$','interpreter','latex')
    
    % figure
    % margin(tsk2cf.L3_i)
    % 
    figure
    lsim(tsk2cf.Q3_i,set.u_i,set.t_vec);
    grid on
    
    figure
    bodemag(tsk2cf.S3_i)
    grid on
    hold on
    bodemag(1/tsk2.W_p_i)
    legend('$S$','$1/W_p$','interpreter','latex')
end
%% TASK 2: Extra

Y_p = ureal('Y_p',Y_p_nomUnc(1),'Perc',Y_p_nomUnc(2));
L_v = ureal('L_v',L_v_nomUnc(1),'Perc',L_v_nomUnc(2));
Y_d = ureal('Y_d',Y_d_nomUnc(1),'Perc',Y_d_nomUnc(2));

A = [Y_v Y_p g; L_v L_p 0; 0 1 0];
B = [Y_d L_d 0]';
C = [0 1 0; 0 0 1; Y_v Y_p 0];
D = [0 0 Y_d]';


model.uncertain = ss(A,B,C,D);
model.uncertain.u = '\delta_{act}';
model.uncertain.y = {'p','\phi','ay'};
model.nominal = model.uncertain.NominalValue;

tsk2rx.R_phi = regulator_tuner(0,tsk2cf.CL3_i.Blocks.R_phi,'e_\phi','p_0');
tsk2rx.R_p = regulator_tuner(0,tsk2cf.CL3_i.Blocks.R_p,{'p_0','p'},'\delta_{lat}');
% il gain dell'integratore va alzato per andare a ridurre l'errore che si 
% crea con il sistema completo, si può evitare di ritunare sul sistema
% completo andando solamente a variare la Ki finchè non diventa robusto.
%tsk2rx.R_p.Ki = 4.01;
R_phi = tsk1r.R_phi;
R_p = tsk1r.R_p;

R_p.u = {'p_0','p'};
Sum = sumblk('e_\phi = \phi_0-\phi');
tsk2x.sys_d_db_nom = tsk2cf.sys_d_db_nom;
tsk2x.sys_d_db_nom.u = '\delta_{lat}';

complete_plant.nominal = connect(tsk2rx.R_phi, tsk2rx.R_p, tsk2x.sys_d_db_nom, set.sys_db_da, ...
    model.nominal,Sum, '\phi_0','\phi',{'\delta_{act}','\delta_{lat}','e_\phi'});

complete_plant.array = usample(model.uncertain,100);
[~,Info] = ucover(complete_plant.array, model.nominal, 3);
tsk2x.W_delta_p = Info.W1;

tsk2x.SumInner1 = sumblk('e_\phi = -\phi');

tsk2x.W_delta_p.u = '\delta_{lat}';
tsk2x.W_delta_p.y = 'z_a';

tsk2x.SumInner2 = sumblk('uili = \delta_{lat} + w_a');

tsk2x.sys_d_db_nom = tsk2x.sys_d_db_nom;
tsk2x.sys_d_db_nom.u = 'uili';

tsk2x.CL4_a = connect(tsk2rx.R_phi, tsk2rx.R_p, tsk2x.sys_d_db_nom, set.sys_db_da, model.nominal, tsk2x.SumInner1, ...
    tsk2x.SumInner2, tsk2x.W_delta_p, 'w_a', 'z_a', {'\delta_{lat}','\phi'});

tsk2x.M4_a = getIOTransfer(tsk2x.CL4_a,'w_a','z_a');

tsk2x.M4_a_fr = bode(tsk2x.M4_a, set.omega_vec);


tsk2x.F4_i = getIOTransfer(complete_plant.nominal,'\phi_0','\phi');
tsk2x.S4_i = getIOTransfer(complete_plant.nominal,'\phi_0','e_\phi');
tsk2x.Q4_i = getIOTransfer(complete_plant.nominal,'\phi_0','\delta_{lat}');
tsk2x.L4_i = (1-tsk2x.S4_i)/(tsk2x.S4_i);

% Robust performance
tsk2x.FW = squeeze(bode(tsk2x.F4_i*tsk2x.W_delta_p,set.omega_vec));
tsk2x.SW = squeeze(bode(tsk2x.S4_i*tsk2c.W_p_i,set.omega_vec));

figure
semilogx(set.omega_vec, (20*log10(tsk2x.FW)+20*log10(tsk2x.SW)));
hold on
semilogx(set.omega_vec,set.omega_vec./set.omega_vec);
grid on 
title('Robust Performance','interpreter','tex','FontWeight','bold')
ylabel('Magnitude (dB)')
xlabel('Frequency (rad/s)')
legend('$|WpS|+|WF|$', '$1$', 'interpreter', 'latex', 'location', 'best')

%%
% figure
% bodemag(tsk2x.W_delta_p, {set.omega_min, set.omega_max},'r')
% hold on 
% bodemag((model.nominal - complete_plant.array)/model.nominal, {set.omega_min,set.omega_max})
% grid on 
%%
if flag.task2plot
    figure 
    semilogx(set.omega_vec, 20*log10(abs(squeeze(tsk2x.M4_a_fr))))
    hold on 
    semilogx(set.omega_vec, 20*log10(set.omega_vec./set.omega_vec))
    grid on 
    legend('$|M_a|$', '$1$', 'interpreter', 'latex', 'location', 'southeast')
    
    
    
    figure
    step(tsk2x.F4_i)
    hold on
    step(set.F_2_i)
    grid on
    legend('Complete System','Target','interpreter','latex','FontSize',13)
    
    stepinfo(tsk2x.F4_i)
    figure
    bodemag(tsk2x.Q4_i)
    grid on
    hold on
    bodemag(1/tsk2c.W_q_i)
    legend('$Q$','$1/W_q$','interpreter','latex')
    
    figure
    margin(tsk2x.L4_i)
    
    figure
    lsim(tsk2x.Q4_i,set.u_i,set.t_vec);
    grid on
    
    figure
    bodemag(tsk2x.S4_i)
    grid on
    hold on
    bodemag(1/tsk2c.W_p_i)
    legend('$S$','$1/W_p$','interpreter','latex')
end

%% Retuning complete

tsk2xR.R_phi = regulator_tuner(1,'R_phi','e_\phi','p_0','P');
tsk2xR.R_p = regulator_tuner(2, 'R_p', {'p_0','p'},{'\delta_{lat}'},'PID');

tsk2xR.W_p_i = tsk2.W_p_i;
%tsk2xR.W_p_i = (s/set.M_i + 12.0)/(s + set.A_i*12.0);
tsk2xR.W_p_i.u = 'e_\phi';
tsk2xR.W_p_i.y = 'z_1';

tsk2xR.W_q_i = tsk2.W_q_i;

R_p.u = {'p_0','p'};
Sum = sumblk('e_\phi = \phi_0-\phi');
tsk2xR.sys_d_db_nom = tsk2cf.sys_d_db_nom;
tsk2xR.sys_d_db_nom.u = '\delta_{lat}';

complete_plantR.nominal = connect(tsk2xR.R_phi, tsk2xR.R_p, tsk2xR.sys_d_db_nom, set.sys_db_da, ...
    model.nominal,Sum, tsk2xR.W_p_i, tsk2xR.W_q_i, '\phi_0',{'z_1','z_2'},{'\delta_{act}','\delta_{lat}','e_\phi','\phi'});

[tsk2xR.CL3_i,~,~] = hinfstruct(complete_plantR.nominal,set.opt);
showTunable(tsk2xR.CL3_i)
%

%
tsk2xR.R_phi = regulator_tuner(0,tsk2xR.CL3_i.Blocks.R_phi,'e_\phi','p_0');
tsk2xR.R_p = regulator_tuner(0,tsk2xR.CL3_i.Blocks.R_p, {'p_0','p'},'\delta_{lat}');

%tuned plant
complete_plantRt.nominal = connect(tsk2xR.R_phi, tsk2xR.R_p, tsk2xR.sys_d_db_nom, set.sys_db_da, ...
    model.nominal,Sum, '\phi_0','\phi',{'\delta_{act}','\delta_{lat}','e_\phi'});

complete_plantR.array = usample(model.uncertain,100);
[~,Info] = ucover(complete_plant.array, model.nominal, 3);
tsk2xR.W_delta_p = Info.W1;

tsk2xR.SumInner1 = sumblk('e_\phi = -\phi');

tsk2xR.W_delta_p.u = '\delta_{lat}';
tsk2xR.W_delta_p.y = 'z_a';

% sensitivity test
S = getIOTransfer(tsk2xR.CL3_i,'\phi_0','e_\phi');
F = getIOTransfer(tsk2xR.CL3_i,'\phi_0','\phi');
w = logspace(-5,5,500);
FW = squeeze(bode(F*tsk2xR.W_delta_p,set.omega_vec));
SW = squeeze(bode(S*tsk2xR.W_p_i,set.omega_vec));
figure
semilogx(set.omega_vec, (20*log10(FW)+20*log10(SW)));
hold on
semilogx(set.omega_vec,set.omega_vec./set.omega_vec);
grid on 
title('Robust Performance','interpreter','tex','FontWeight','bold')
ylabel('Magnitude (dB)')
xlabel('Frequency (rad/s)')
legend('$|WpS|+|WF|$', '$1$', 'interpreter', 'latex', 'location', 'best')


tsk2xR.SumInner2 = sumblk('uili = \delta_{lat} + w_a');

tsk2xR.sys_d_db_nom = tsk2xR.sys_d_db_nom;
tsk2xR.sys_d_db_nom.u = 'uili';

tsk2xR.CL4_a = connect(tsk2xR.R_phi, tsk2xR.R_p, tsk2xR.sys_d_db_nom, set.sys_db_da, model.nominal, tsk2xR.SumInner1, ...
    tsk2xR.SumInner2, tsk2xR.W_delta_p, 'w_a', 'z_a', {'\delta_{lat}','\phi'});

tsk2xR.M4_a = getIOTransfer(tsk2xR.CL4_a,'w_a','z_a');

tsk2xR.M4_a_fr = bode(tsk2xR.M4_a, set.omega_vec);


tsk2xR.F4_i = getIOTransfer(complete_plantRt.nominal,'\phi_0','\phi');
tsk2xR.S4_i = getIOTransfer(complete_plantRt.nominal,'\phi_0','e_\phi');
tsk2xR.Q4_i = getIOTransfer(complete_plantRt.nominal,'\phi_0','\delta_{lat}');
tsk2xR.L4_i = (1-tsk2xR.S4_i)/(tsk2xR.S4_i);

%%
%if flag.task2plot
    figure 
    semilogx(set.omega_vec, 20*log10(abs(squeeze(tsk2xR.M4_a_fr))))
    hold on 
    semilogx(set.omega_vec, 20*log10(set.omega_vec./set.omega_vec))
    grid on 
    title('Robust Stability','interpreter','tex','FontWeight','bold')
    ylabel('Magnitude (dB)')
    xlabel('Frequency (rad/s)')
    legend('$|M_a|$', '$1$', 'interpreter', 'latex', 'location', 'southeast')
    
    
    
    figure
    step(tsk2xR.F4_i)
    hold on
    step(set.F_2_i)
    grid on
    legend('Complete System','Target','interpreter','latex')
    
    stepinfo(tsk2xR.F4_i)
    figure
    bodemag(tsk2xR.Q4_i)
    grid on
    hold on
    bodemag(1/tsk2xR.W_q_i)
    legend('$Q$','$1/W_q$','interpreter','latex')
    
    % figure
    % margin(tsk2xR.L4_i)
    
    figure
    lsim(tsk2xR.Q4_i,set.u_i,set.t_vec);
    grid on
    
    figure
    bodemag(tsk2xR.S4_i)
    grid on
    hold on
    bodemag(1/tsk2xR.W_p_i)
    legend('$S$','$1/W_p$','interpreter','latex')
%end
