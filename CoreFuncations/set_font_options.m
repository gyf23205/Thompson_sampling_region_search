function set_font_options(handle)
if nargin == 0
    handle = gca;
end

set(handle, 'FontName', 'bitstream charter', ...
           'FontSize', 14, ...
           'FontWeight', 'bold');
