%clear

%% MONTE CARLO
warning('off','all')
Y_v_nomUnc = [-0.1068   , 3*4.26];  % [1/s]        -> 4.26 % - nomUnc = nominal, uncertainty
Y_p_nomUnc = [0.1192    , 3*2.03];  % [m/(s rad)]  -> 2.03 %
L_v_nomUnc = [-5.9755   , 3*1.83];  % [rad s/m]    -> 1.83 %
L_p_nomUnc = [-2.6478   , 3*2.01];  % [1/s]        -> 2.01 %
Y_d_nomUnc = [-10.1647  , 3*1.37];  % [m/s^2]      -> 1.37 %
L_d_nomUnc = [450.7085  , 3*0.81];  % [rad/s^2]    -> 0.81 %
K_v_nomUnc = [0.9999    , 0.75];
T_a_nomUnc = [0.02      , 0];

%%% Stability Derivatives
Y_v = ureal('Y_v',Y_v_nomUnc(1),'Perc',Y_v_nomUnc(2));
Y_p = ureal('Y_p',Y_p_nomUnc(1),'Perc',Y_p_nomUnc(2));
L_p = ureal('L_p',L_p_nomUnc(1),'Perc',L_p_nomUnc(2));
L_v = ureal('L_v',L_v_nomUnc(1),'Perc',L_v_nomUnc(2));

%%% Control Derivatives
Y_d = ureal('Y_d',Y_d_nomUnc(1),'Perc',Y_d_nomUnc(2));
L_d = ureal('L_d',L_d_nomUnc(1),'Perc',L_d_nomUnc(2));
K_v = ureal('K_v', K_v_nomUnc(1), 'Range', [K_v_nomUnc(2),1]);

set.t_min = 0;                                      % [s]
set.t_max = 6;                                      % [s]
set.t_vec = linspace(set.t_min,set.t_max,10^4);

set.u_i = 0*(set.t_vec<=1) + 10*(set.t_vec>1 & set.t_vec<= 3) - 10*(set.t_vec>3 & set.t_vec<= 5) + 0*(set.t_vec>5);
set.u_i = deg2rad(set.u_i);

load("src/startMonte.mat")
%tsk3.R_phi.Kp = 8;
%%
mnt.F_nom_i = getIOTransfer(obs_plant.nominal,'\phi_0','\phi');
mnt.S_nom_i = getIOTransfer(obs_plant.nominal, '\phi_0', 'e_\phi');
mnt.Q_nom_i = getIOTransfer(obs_plant.nominal,'\phi_0','\delta_{lat}');
mnt.L_nom_i = (1-mnt.S_nom_i)/mnt.S_nom_i;

mnt.st_nom_int = stepinfo(mnt.F_nom_i).SettlingTime;
mnt.ov_nom_int = stepinfo(mnt.F_nom_i).Overshoot;
mnt.max_nom_int = max(abs(lsim(mnt.Q_nom_i,set.u_i,set.t_vec)));

[mnt.gm_nom_int, mnt.pm_nom_int] = margin(mnt.L_nom_i);

N = 10000;
N_histCol = 100; 
%%
Y_v_vec = zeros(N,1);
Y_p_vec = zeros(N,1);
L_p_vec = zeros(N,1);
L_v_vec = zeros(N,1);
Y_d_vec = zeros(N,1);
L_d_vec = zeros(N,1);
K_v_vec = zeros(N,1);

st_int = zeros(N,1);
ov_int = zeros(N,1);
pm_int = zeros(N,1);
gm_int = zeros(N,1);
max_int = zeros(N,1);

st_m_int = zeros(N,1);
ov_m_int = zeros(N,1);
pm_m_int = zeros(N,1);
gm_m_int = zeros(N,1);
max_m_int = zeros(N,1);

parfor i=1:N

    i
    % Stability Derivatives
    Y_v_vec(i) = usample(Y_v);
    Y_p_vec(i) = usample(Y_p);
    L_p_vec(i) = usample(L_p);
    L_v_vec(i) = usample(L_v);
    
    % Control Derivatives
    Y_d_vec(i) = usample(Y_d);
    L_d_vec(i) = usample(L_d);
    K_v_vec(i) = usample(K_v);

    % State Space Model
    A = [Y_v_vec(i) Y_p_vec(i) 9.81 ; L_v_vec(i) L_p_vec(i) 0 ; 0 1 0 ];
    B = [Y_d_vec(i) L_d_vec(i) 0 ]';
    C = [0 1 0; 0 0 1; Y_v_vec(i) Y_p_vec(i) 0];
    D = [0 0 Y_d_vec(i)]';
    sys_d_db = tf(K_v_vec(i), 1);
    sys_d_db.u = '\delta_{lat}';
    sys_d_db.y = '\delta_{bat}';
    sys_db_da = tf(1,[0.02, 1]);
    sys_db_da.u = '\delta_{bat}';
    sys_db_da.y = '\delta_{act}';
    sys_ld = ss(A,B,C,D);
    sys_ld.u = '\delta_{act}';
    sys_ld.y = {'p','\phi','ay'};
    
    % summation block
    SumInner1 = sumblk('e_\phi = \phi_0 - \phi_{hat}');
    

    % blocks connection
    CLmont_i = connect(tsk3.R_phi, tsk3.R_p, sys_d_db, sys_db_da,...
        sys_ld, SumInner1,tsk3.sp, tsk3.Obs, '\phi_0','\phi',...
        {'\delta_{act}','\delta_{lat}','\phi_{hat}','p_{hat}','p','e_\phi'});
           
    % retrieve sensitivity functions
    Fmont_i = getIOTransfer(CLmont_i,'\phi_0','\phi');
    Smont_i = getIOTransfer(CLmont_i,'\phi_0','e_\phi');
    Qmont_i = getIOTransfer(CLmont_i,'\phi_0','\delta_{lat}');
    Lmont_i = (1-Smont_i)/Smont_i;

    % save relevant parameters
    max_int(i) = max(abs(lsim(Qmont_i,set.u_i,set.t_vec))); % 
    st_int(i) = stepinfo(Fmont_i).SettlingTime;     % settling time
    ov_int(i) = stepinfo(Fmont_i).Overshoot;        % overshoot
    [gm_int(i), pm_int(i)] = margin(Lmont_i);       % gain margin and phase margin

end

st_m_int(1) = st_int(1);
ov_m_int(1) = ov_int(1);
max_m_int(1) = max_int(1);
gm_m_int(1) = gm_int(1);
pm_m_int(1) = pm_int(1);

for i = 2:N
    st_m_int(i) = (st_m_int(i-1)*(i-1) + st_int(i))/i; % much faster way to compute the means, bad side: if N > 500000 the numerical method brakes due to computational errors (but the gain in speed is significant)
    ov_m_int(i) = (ov_m_int(i-1)*(i-1) + ov_int(i))/i;
    max_m_int(i) = (max_m_int(i-1)*(i-1) + max_int(i))/i;
    gm_m_int(i) = (gm_m_int(i-1)*(i-1) + gm_int(i))/i;
    pm_m_int(i) = (pm_m_int(i-1)*(i-1) + pm_int(i))/i;
end

montecarloResultsInnerLoop = struct('max_int',max_int,'st_int',st_int,'ov_int',ov_int,...
    'pm_int',pm_int,'gm_int',gm_int);
if isunix
    save("MontecarloResults/innerLoop_N"+num2str(N),"montecarloResultsInnerLoop");
elseif ispc
    save("MontecarloResults\innerLoop_N"+num2str(N),"montecarloResultsInnerLoop");
end
%% Plots
if isunix
    load("MontecarloResults/innerLoop_N"+num2str(N)+".mat")
elseif ispc
    load("MontecarloResults\innerLoop_N"+num2str(N)+".mat")
end
%%

fprintf('*************** MONTE CARLO ***************\n');
fprintf('Number of samples: [%d]\n', N);
fprintf('## Step Info: \n');
fprintf('Mean Value:            [%2.6f] s  \n', mean(st_int));
fprintf('Standard Deviation:    [%2.6f]  \n', std(st_int));
fprintf('\n## Control Effort: \n');
fprintf('Mean Value:            [%2.6f] deg  \n', mean(max_int));
fprintf('Standard Deviation:    [%2.6f]  \n', std(max_int));
fprintf('\n## Phase Margin: \n');
fprintf('Mean Value:            [%2.6f] deg  \n', mean(pm_int));
fprintf('Standard Deviation:    [%2.6f]  \n', std(pm_int));

% Montecarlo results  settling time
figure
subplot(2,1,1)
histogram(st_int,N_histCol)
hold on
xline(mean(st_int), 'r--', 'LineWidth', 1,'Label','Mean Value','LabelOrientation','horizontal')
grid on
title('Settling Time','interpreter','tex','FontWeight','bold')
xlabel('Time (seconds)','interpreter','tex','FontWeight','bold')

subplot(2,1,2)
plot(st_m_int, '-', 'LineWidth', 1)
hold on
yline(mean(st_int), '--', 'LineWidth', 1)
ylabel('Time (seconds)','interpreter','tex','FontWeight','bold')
xlabel('Iterations','interpreter','tex','FontWeight','bold')
grid on
box on

% Montecarlo results  overshoot
figure
histogram(ov_int,N_histCol)
hold on
grid on
title('Overshoot','interpreter','tex','FontWeight','bold')
%title(['Overshoot, target value = ', num2str(stepinfo(F_2_i).Overshoot)])

% subplot(2,1,2)
% plot(ov_m_int, '-', 'LineWidth', 1)
% hold on
% yline(mnt.ov_nom_int, '--', 'LineWidth', 1)
% grid on
% box on

% Montecarlo results  control effort
figure
subplot(2,1,1)
histogram(max_int,N_histCol)
hold on
xline(mean(max_int), 'r--', 'LineWidth', 1,'Label','Mean Value','LabelOrientation','horizontal')
grid on
title('Control effort, limit value = 5','interpreter','tex','FontWeight','bold')

subplot(2,1,2)
plot(max_m_int, '-', 'LineWidth', 1)
hold on
yline(mean(max_int), '--', 'LineWidth', 1)
xlabel('Iterations','interpreter','tex','FontWeight','bold')
grid on
box on

% Montecarlo results  phase margin
figure
subplot(2,1,1)
histogram(pm_int,N_histCol)
hold on
xline(mnt.pm_nom_int, 'r--', 'LineWidth', 1)
grid on
title('Phase margin')

subplot(2,1,2)
plot(pm_m_int, '-', 'LineWidth', 1)
hold on
yline(mnt.pm_nom_int, '--', 'LineWidth', 1)
grid on
box on