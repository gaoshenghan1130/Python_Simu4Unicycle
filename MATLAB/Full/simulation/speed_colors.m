function colors=speed_colors(speeds)
% Sequential blue map: larger speed is always shown with a darker line.
speeds=speeds(:);
low_color=[0.68 0.84 0.98];
high_color=[0.02 0.12 0.36];
if numel(speeds)==1 || max(speeds)==min(speeds)
    level=ones(size(speeds));
else
    level=(speeds-min(speeds))/(max(speeds)-min(speeds));
end
colors=low_color+(high_color-low_color).*level;
end
