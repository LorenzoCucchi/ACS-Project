%{
    Aerospace Control Systems:
    Robust stability and performance analysis of a quadrotor drone.

    task0

    Formulation of the problem and data definition

    authors: ...

%}

%% DATA

%%% System definition
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
Y_p = Y_p_nomUnc(1); % considered nominal in design phase
L_p = ureal('L_p',L_p_nomUnc(1),'Perc',L_p_nomUnc(2));
L_v = L_v_nomUnc(1); % considered nominal in design phase

%%% Control Derivatives
Y_d = Y_d_nomUnc(1);
L_d = ureal('L_d',L_d_nomUnc(1),'Perc',L_d_nomUnc(2));

%%% Motor Plant
K_v = ureal('K_v', K_v_nomUnc(1), 'Range', [K_v_nomUnc(2),1]);
set.sys_d_db = tf(K_v, 1);
set.sys_d_db.u = '\delta_{lat}';
set.sys_d_db.y = '\delta_{bat}';
set.sys_d_db_nom = set.sys_d_db.NominalValue;
set.sys_d_db_nom.u = '\delta_{lat}';
set.sys_d_db_nom.y = set.sys_d_db.y;

set.sys_db_da = tf(1,[T_a_nomUnc(1), 1]);
set.sys_db_da.u = '\delta_{bat}';
set.sys_db_da.y = '\delta_{act}';

%%% constants
g = 9.81;

%%% SISO from "delta" to "p" with G_delta_p = L_d/(s-L_p)
plants.delta_p.A = L_p;
plants.delta_p.B = L_d;
plants.delta_p.C = 1;
plants.delta_p.D = 0;

%%% SISO from "p" to "phi" with G_p_phi = 1/s;
plants.p_phi.A=0;
plants.p_phi.B=1;
plants.p_phi.C=1;
plants.p_phi.D=0;

% SISO from phi to v with G_phi_v = g/(s-Y_v)
plants.phi_v.A = Y_v;
plants.phi_v.B = g;
plants.phi_v.C = 1;
plants.phi_v.D = 0;

%%% SISO from v to y with G_v_y = 1/s;
plants.v_y.A = 0;
plants.v_y.B = 1;
plants.v_y.C = 1;
plants.v_y.D = 0;



%% System generation

[set.sys_delta_p,set.sys_delta_p_nom] = system_gen(plants.delta_p,'\delta_{lat}','p');
[set.sys_p_phi,set.sys_p_phi_nom]     = system_gen(plants.p_phi,'p','\phi');
[set.sys_phi_v,set.sys_phi_v_nom]     = system_gen(plants.phi_v,'\phi','v');
[set.sys_v_y,set.sys_v_y_nom]         = system_gen(plants.v_y,'v','y');


%% Plant Analysis
G_delta_p_nom   = tf(set.sys_delta_p_nom);
G_delta_p_unc   = tf(set.sys_delta_p);
z_delta_p       = tzero(set.sys_delta_p_nom);
p_delta_p       = pole(set.sys_delta_p_nom);

G_p_phi_nom     = tf(set.sys_p_phi_nom);
z_p_phi         = tzero(set.sys_p_phi_nom);
p_p_phi         = pole(set.sys_p_phi_nom);

G_phi_v_nom     = tf(set.sys_phi_v_nom);
z_phi_v         = tzero(set.sys_phi_v_nom);
p_phi_v         = pole(set.sys_phi_v_nom);

G_v_y_nom       = tf(set.sys_v_y_nom);
z_v_y           = tzero(set.sys_v_y_nom);
p_v_y           = pole(set.sys_v_y_nom);

if flag.PlantAnalysisImages
   
    plotoptions = pzoptions('cstprefs');
    fig = figure;
    h = pzplot(set.sys_delta_p, plotoptions);
    hold on
    pzplot(set.sys_delta_p_nom, plotoptions);
    grid on
    legend('Uncertain model','Nominal model','location','best', 'FontSize', 14)
    

    figure 
    bode(set.sys_delta_p);
    hold on
    bode(set.sys_delta_p_nom);
    grid on
    legend('Uncertain model','Nominal model', 'FontSize', 14);

    figure
    pzplot(set.sys_phi_v);
    hold on
    pzplot(set.sys_phi_v_nom);
    grid on
    legend('Uncertain model','Nominal model','location','east', 'FontSize', 14)

    figure 
    bode(set.sys_phi_v);
    hold on
    bode(set.sys_phi_v_nom);
    grid on
    legend('Uncertain model','Nominal model', 'FontSize', 14);

end

