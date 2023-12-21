%% OBSERVER 

tsk3.A = A.NominalValue;
tsk3.B = B.NominalValue;
tsk3.C = C.NominalValue;
tsk3.D = D.NominalValue;

tsk3.C_obs = tsk3.C([1 3],:);
tsk3.D_obs = tsk3.D([1,3]);

% hand placing of poles, not nice
tsk3.pmin = -max(abs(eig(tsk3.A)));
%tsk3.P = [80*tsk3.pmin -0.01 71*tsk3.pmin]';
tsk3.P = [-0.5 -0.01 -10]';
tsk3.Lt = place(tsk3.A',tsk3.C_obs',tsk3.P);
tsk3.L = tsk3.Lt';

% LQR placing of poles, very nice
% tsk3.Q = [1000 0 0; 0 1001 0; 0 0 0.0005];
% % tsk3.Q = diag([200*Y_v_uncertainty^2, 200*Y_p_uncertainty^2, 1/L_v_uncertainty^2]);
% tsk3.R = [0.01 0; 0 1e6];
% % tsk3.R = diag([1/Y_d_uncertainty^2, 150*L_d_uncertainty^2]);
% [tsk3.K, ~, ~] = lqr(tsk3.A',tsk3.C_obs',tsk3.Q,tsk3.R);
% tsk3.L = tsk3.K';

tsk3.A_hat = tsk3.A - tsk3.L*tsk3.C_obs;
tsk3.eigenvalue_observer = eig(tsk3.A_hat);
tsk3.nat_freq_obs = abs(imag(tsk3.eigenvalue_observer));
tsk3.B_hat = [(tsk3.B - tsk3.L*tsk3.D_obs) tsk3.L];
tsk3.C_hat = tsk3.C;
%tsk3.D_hat = [tsk3.D zeros(3,2)];
tsk3.D_hat = zeros(3,3);


tsk3.Obs = ss(tsk3.A_hat, tsk3.B_hat, tsk3.C_hat, tsk3.D_hat);
tsk3.Obs.u = {'\delta_{act}','p','ay'};
tsk3.Obs.y = {'p_{hat}','\phi_{hat}','ay_{hat}'};

tsk3.R_phi = tsk2xR.R_phi;
tsk3.R_p = tsk2xR.R_p;
tsk3.sp = sumblk('p_2 = p - p_{hat} + p_{hat}');
tsk3.R_p.u = {'p_0','p_2'};
tsk3.Sum = sumblk('e_\phi = \phi_0 - \phi_{hat}');

tsk3.R_phi
tsk3.R_p
tsk3.sys_d_db_nom = tsk2x.sys_d_db_nom;
tsk3.sys_d_db_nom.u = '\delta_{lat}';

obs_plant.uncertain = connect(tsk3.R_phi, tsk3.R_p, set.sys_d_db, set.sys_db_da, model.uncertain, tsk3.Sum,tsk3.sp, tsk3.Obs, '\phi_0','\phi',{'\delta_{act}','\delta_{lat}','\phi_{hat}','p_{hat}','p','e_\phi'});

obs_plant.nominal = connect(tsk3.R_phi, tsk3.R_p, tsk3.sys_d_db_nom, set.sys_db_da, model.nominal, tsk3.Sum,tsk3.sp, tsk3.Obs, '\phi_0','\phi',{'\delta_{act}','\delta_{lat}','\phi_{hat}','p_{hat}','p','e_\phi'});

save src/startMonte obs_plant tsk3;

obs_plant.array = usample(obs_plant.uncertain,100);
figure
step(obs_plant.array)
grid on
hold on

[~,Info] = ucover(obs_plant.array, obs_plant.sys_d_p_nom, 3);
tsk3.W_delta_p = Info.W1;



%%
tsk3.Q = getIOTransfer(obs_plant.nominal,'\phi_0','\delta_{lat}');
figure
lsim(tsk3.Q,set.u_i,set.t_vec);
grid on;
%%
tsk3.F = getIOTransfer(obs_plant.nominal,'\phi_0','\phi');
tsk3.F_est = getIOTransfer(obs_plant.nominal,'\phi_0','\phi_{hat}');

tsk3.G = getIOTransfer(obs_plant.nominal,'\phi_0','p');
tsk3.G_est = getIOTransfer(obs_plant.nominal,'\phi_0','p_{hat}');

tsk3.S = getIOTransfer(obs_plant.nominal,'\phi_0','e_\phi');

figure
step(tsk3.F)
hold on
step(set.F_2_i)
legend('Complete System','Target')

t = linspace(0,7,10^4);
u = 0*(t<=1) + 10*(t>1 & t<= 3) - 10*(t>3 & t<= 5) + 0*(t>5);
u = deg2rad(u);

figure()
hold on 
yyaxis left
lsim(tsk3.F,u,t,'b');
lsim(tsk3.F_est,u,t,'r--');
grid on 
legend ('reale','osservato','interpreter','latex');
title('roll angle')

figure
lsim(tsk3.Q,set.u_i,set.t_vec);
hold on
grid on

figure
bodemag(tsk3.S)
grid on
hold on
bodemag(1/tsk2xR.W_p_i)
legend('$S$','$1/W_p$','interpreter','latex')


figure
bodemag(tsk3.Q)
grid on
hold on
bodemag(1/tsk2xR.W_q_i)
legend('$Q$','$1/W_q$','interpreter','latex')

%plot(set.t_vec,y,'Color',"#EDB120")
% figure()
% hold on 
% lsim(tsk3.G,u,t,'b');
% lsim(tsk3.G_est,u,t,'r--');
% grid on 
% legend ('reale','osservato','interpreter','latex');

tsk3.SumInner1 = sumblk('e_\phi = -\phi_{hat}');
tsk3.SumInner2 = sumblk('uili = \delta_{lat} + w_a');

tsk33.sys_d_db_nom = tsk3.sys_d_db_nom;
tsk33.sys_d_db_nom.u = 'uili';

tsk3.CL3_a = connect(tsk3.R_phi, tsk3.R_p, tsk33.sys_d_db_nom, set.sys_db_da, model.nominal, ...
    tsk3.Obs, tsk3.SumInner1, tsk3.SumInner2, tsk2x.W_delta_p, 'w_a', 'z_a', {'\delta_{lat}','\phi_{hat}'});

tsk3.M_a = getIOTransfer(tsk3.CL3_a,'w_a','z_a');

tsk3.M_a_fr = bode(tsk3.M_a, set.omega_vec);

figure
semilogx(set.omega_vec, 20*log10(abs(squeeze(tsk3.M_a_fr))))
hold on 
semilogx(set.omega_vec, 20*log10(set.omega_vec./set.omega_vec))
grid on 
legend('$|M_a|$','$1$','interpreter','latex')

% Robust performance
tsk3.FW = squeeze(bode(tsk3.F*tsk2x.W_delta_p,set.omega_vec));
tsk3.SW = squeeze(bode(tsk3.S*tsk2c.W_p_i,set.omega_vec));

figure
semilogx(set.omega_vec, (20*log10(tsk3.FW)+20*log10(tsk3.SW)));
hold on
semilogx(set.omega_vec,set.omega_vec./set.omega_vec);
grid on 
title('Robust Performance','FontWeight','bold')
ylabel('Magnitude (dB)')
xlabel('Frequency (rad/s)')
legend('$|WpS|+|WF|$', '$1$', 'interpreter', 'latex', 'location', 'best')
