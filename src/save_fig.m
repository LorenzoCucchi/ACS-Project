function save_fig(fig,name)
% save_fig - Function to save figure to pdf with correct sizing.
%
% PROTOTYPE
%   [t, Y, ct] = BDF3(f, tlim, h, x0)
%
% INPUT:
%   fig      figure      [1x1]   figure to save                         [-]
%   name     double      [1x2]   str                                    [-]
%
% CONTRIBUTOR:
%   Cucchi Lorenzo              10650070
% -------------------------------------------------------------------------
WHratio = fig.Position(3)/fig.Position(4); % retrieve current WHratio
widthPos = 15;
heightPos = widthPos/WHratio;

set(fig,'Units','centimeters',...
       'PaperUnits','centimeters',...
       'PaperSize',[widthPos heightPos],...
       'PaperPositionMode','auto',...
       'InvertHardcopy', 'on');
name = strcat('figures\',name);
saveas(fig,name,'pdf')
%close(fig)
end