function [omega_b_opt,alpha_opt,optimal_param] = innerLoopOptimization(alpha_vec,omega_b_vec,dim,settings)

% HELP
%
% INPUT:
% M,A,alpha,w_b_vec vec are vectors which contain parameters to cycle on
%  is the maximum value of w_b obtainable

% recall usefull parameters
M = settings.M_i;
A = settings.A_i;
sys_delta_p_nom = settings.sys_delta_p_nom;
sys_p_phi_nom = settings.sys_p_phi_nom;
sys_delta_p = settings.sys_delta_p;
omega_vec = settings.omega_vec;
omega_tau = settings.omega_tau_i;

s = tf('s');

%parallel computing parameters inizialization
par_opt = zeros(dim,dim);

% START OPTIMIZATION
parfor k=1:dim
    sys_delta_p_nom_parf = sys_delta_p_nom;

    omega_b = omega_b_vec(k);
    for l=1:dim
[k,l]
        alpha = alpha_vec(l);

        % controller (phi) definition
        R_phi = regulator_tuner(1,'R_phi','e_\phi','p_0','P')

        % controller (p) definition
        R_p = regulator_tuner(2,'R_p',{'p_0','p'},{'\delta_{lat}'},'PID')

        SumInner1 = sumblk('e_\phi = \phi_0 - \phi');

        W_p = (s/M + omega_b)/(s + A*omega_b);
        W_q = alpha*(s+omega_tau*1e-3)/(s+omega_tau);

        % Structured Hinf synthesis

        W_p.u = 'e_\phi';
        W_p.y = 'z_1';

        W_q.u = '\delta_{lat}';
        W_q.y = 'z_2';

        sys_delta_p_nom_parf.u = '\delta_{lat}';

        % augmented plant for Nominal Performance and Control
        % Effort Moderation
        CL0 = connect(R_phi, R_p, sys_p_phi_nom, sys_delta_p_nom_parf, SumInner1, W_p, W_q,...
            '\phi_0',{'z_1','z_2'},{'\delta_{lat}','e_\phi','\phi'});


        [CL,~,~] = hinfstruct(CL0,settings.opt);

        S = getIOTransfer(CL,'\phi_0','e_\phi');
        Q = getIOTransfer(CL,'\phi_0','\delta_{lat}');

        if max(abs(bode(S*W_p,omega_vec))) < 1 && max(abs(bode(Q*W_q,omega_vec))) < 1

            sys_delta_p_array = usample(sys_delta_p,100);
            try
                [~,Info] = ucover(sys_delta_p_array,sys_delta_p_nom_parf,3);
            catch
                par_opt(k,l) = 0;
                continue
            end

            W_delta_p = Info.W1;

            % Controllori con parametri fissati

            R_phi=regulator_tuner(0,CL.Blocks.R_phi,'e_\phi','p_0');
            SumInner1 = sumblk('e_\phi = - \phi');

            R_p=regulator_tuner(0,CL.Blocks.R_p,{'p_0','p'},'\delta_{lat}');
      
            W_delta_p.u = '\delta_{lat}';
            W_delta_p.y = 'z_a';

            SumInner2 = sumblk('uili = \delta_{lat} + w_a');

            sys_delta_p_nom_parf.u = 'uili';

            CL_a = connect(R_phi, R_p, sys_p_phi_nom, sys_delta_p_nom_parf, SumInner1, SumInner2, W_delta_p,...
                'w_a','z_a',{'\delta_{lat}','\phi'});

            M_a = getIOTransfer(CL_a,'w_a','z_a');

            if max(abs(bode(M_a,omega_vec)))<1
                par_opt(k,l) = max(abs(bode(S*W_p,omega_vec))) + ...
                    max(abs(bode(Q*W_q,omega_vec))) + max(abs(bode(M_a,omega_vec)));
            end
        end
    end
end

% save maximum value (less stringent that satisfies parameters):
[~, maxidx] = max(par_opt(:));
[kmax, lmax] = ind2sub( size(par_opt), maxidx );

% check for code mistakes:
if max(max(par_opt)) ~= par_opt(kmax,lmax)
    error('the indeces are wrong')
end

% Optimal parameters:
omega_b_opt = omega_b_vec(kmax);
alpha_opt = alpha_vec(lmax);
optimal_param.matrix = par_opt;
optimal_param.best_value = par_opt(kmax, lmax);


