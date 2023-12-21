function [R_phi, R_p] = controller_creator()

    R_phi = tunablePID('R_phi', 'P');
    R_phi.u = 'e';
    R_phi.y = 'p_0';

    R_p = tunablePID2('R_p', 'PID');
    R_p.c.Value = 0;
    R_p.c.Free = false;
    R_p.b.Value = 1;
    R_p.b.Free = false;
    R_p.Tf.Value = 0.01;
    R_p.Tf.Free = false;
    R_p.u = {'p_0', 'p'};
    R_p.y = {'\delta'};

end