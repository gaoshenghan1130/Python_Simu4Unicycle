clear; clc; close all;

root = fileparts(mfilename('fullpath'));
input_file = fullfile(root,'friction_data.mat');

if ~isfile(input_file)
    error('Cannot find friction_data.mat in the script folder.');
end

S = load(input_file);
R = array2table(S.data,'VariableNames',cellstr(string(S.column_names(:)))); 
mass_kg = S.mass_kg; 

exclude_warmup = true; 
min_speed_mps = 0.002; 
position_bin_mm = 5; 
velocity_bin_mmps = 10; 
min_samples_per_bin = 20; 

assert(all(diff(R.t_s)>0),'Time must increase within one run.'); 
assert(abs(median(diff(R.t_s))-0.01)<0.001,...
    'Select raw 10 ms data, not averaged data.'); 
 
%% Raw 10 ms acceleration
A = R; 
A.a_mps2(:) = NaN; 

dt = diff(R.t_s); 
ok = abs(dt-0.01)<0.001; 
a = diff(R.v_mps)./dt; 
a(~ok) = NaN; 

A.a_mps2(2:end) = a; 
A.t_s(2:end) = (R.t_s(1:end-1)+R.t_s(2:end))/2; 
 
%% 100 ms difference acceleration
B = R; 
fields = {'rmean_m','vmean_mps','Fmean_N','a_mps2','vref_mps'}; 

for k = 1:numel(fields)
    B.(fields{k})(:) = NaN; 
end 

for i = 11:height(R) 
    j = i-10; 
    
    meta = R{j:i,{'warmup','speed_index','stroke','dir'}}; 
    
    if any(any(diff(meta,1,1)~=0)) || ...
            any(abs(diff(R.t_s(j:i))-0.01)>0.001) 
        continue; 
    end 
    
    w = diff(R.t_s(j:i)); 
    span = sum(w); 
    
    B.t_s(i) = (R.t_s(j)+R.t_s(i))/2; 
    B.a_mps2(i) = (R.v_mps(i)-R.v_mps(j))/span; 
    
    for k = [1 2 3 5] 
        field = fields{k}; 
        B.(field)(i) = sum(R.(field)(j+1:i).*w)/span; 
    end 
end 
 
%% Plot both processing methods
groups = {A,B}; 
group_names = {'raw_10ms','difference_100ms'}; 

for g = 1:2 
    
    T = groups{g}; 
    T.Festimate_N = T.Fmean_N - mass_kg .* T.a_mps2; 
    
    keep = all(isfinite(T{:, ...
        {'rmean_m','vmean_mps','Festimate_N'}}), 2); 
    
    if exclude_warmup 
        keep = keep & T.warmup == 0; 
    end 
    
    D = T(keep, :); 
    M = D(abs(D.vmean_mps) >= min_speed_mps, :); 
    
    if isempty(M)
        warning('No map data for %s',group_names{g});
        continue;
    end 
 
    r = M.rmean_m * 1000; 
    v = M.vmean_mps * 1000; 
    f = M.Festimate_N; 
    
    cmax = max(abs(f)); 
    if cmax == 0
        cmax = 1;
    end 
 
    fprintf(['Group: ' group_names{g} '\n']); 
    fprintf('All records: %d; selected: %d; moving map records: %d\n', ... 
        height(T), height(D), height(M)); 
    fprintf('Mass used for acceleration correction: %.3f kg\n', mass_kg); 
    fprintf('Map uses measured velocity, not target speed.\n'); 
    fprintf('Signed estimate: Fcmd - m*a; positive means required positive drive force.\n'); 
    fprintf('Near-zero-speed points remain in time plots, but are excluded from maps.\n'); 
 
    %% Figure 1: 3D friction
    fig1 = figure('Color','w','Name','Friction vs position and velocity'); 
    
    scatter3(r, v, f, 20, f, 'filled'); 
    
    xlabel('Position r (mm)'); 
    ylabel('Measured velocity v (mm/s)'); 
    zlabel('Estimated friction (N)'); 
    title('Measured samples: F_{cmd} - m a'); 
    
    grid on; 
    box on; 
    view(45,25); 
    
    colormap(fig1, parula); 
    colorbar; 
    caxis([-cmax cmax]); 
 
    %% Binning
    re = floor(min(r)/position_bin_mm)*position_bin_mm : ...
        position_bin_mm : ...
        (ceil(max(r)/position_bin_mm)+1)*position_bin_mm;
    
    ve = floor(min(v)/velocity_bin_mmps)*velocity_bin_mmps : ...
        velocity_bin_mmps : ...
        (ceil(max(v)/velocity_bin_mmps)+1)*velocity_bin_mmps;
    
    ri = discretize(r, re); 
    vi = discretize(v, ve); 
    
    nr = numel(re)-1; 
    nv = numel(ve)-1; 
    
    F = nan(nv,nr); 
    N = zeros(nv,nr); 
    
    for j = 1:nv 
        for i = 1:nr 
            q = ri == i & vi == j; 
            N(j,i) = nnz(q); 
            
            if N(j,i) >= min_samples_per_bin 
                F(j,i) = median(f(q)); 
            end 
        end 
    end 
    
    rc = (re(1:end-1)+re(2:end))/2; 
    vc = (ve(1:end-1)+ve(2:end))/2; 
 
    %% Figure 2: friction map
    fig2 = figure('Color','w','Name','Binned friction map'); 
    
    tiledlayout(1,2,'TileSpacing','compact'); 
    
    nexttile; 
    h = imagesc(rc,vc,F); 
    set(h,'AlphaData',isfinite(F)); 
    set(gca,'YDir','normal','Color',[0.92 0.92 0.92]); 
    
    xlabel('Position r (mm)'); 
    ylabel('Measured velocity v (mm/s)'); 
    title(sprintf('Median friction (N), at least %d samples/bin',...
        min_samples_per_bin)); 
    
    colorbar; 
    caxis([-cmax cmax]); 
    
    nexttile; 
    imagesc(rc,vc,N); 
    set(gca,'YDir','normal'); 
    
    xlabel('Position r (mm)'); 
    ylabel('Measured velocity v (mm/s)'); 
    title('Samples per bin'); 
    
    colorbar; 
    colormap(fig2,parula); 
 
    %% Figure 3: projections
    fig3 = figure('Color','w','Name','Friction projections'); 
    
    tiledlayout(1,2,'TileSpacing','compact'); 
    
    nexttile; 
    scatter(r,f,16,v,'filled'); 
    
    xlabel('Position r (mm)'); 
    ylabel('Estimated friction (N)'); 
    title('Color: measured velocity (mm/s)'); 
    
    colorbar; 
    grid on; 
    
    nexttile; 
    scatter(v,f,16,r,'filled'); 
    
    xlabel('Measured velocity v (mm/s)'); 
    ylabel('Estimated friction (N)'); 
    title('Color: position (mm)'); 
    
    colorbar; 
    grid on; 
    
    colormap(fig3,parula); 
 
    %% Figure 4: history
    fig4 = figure('Color','w','Name','Motion and force history'); 
    
    tiledlayout(3,1,'TileSpacing','compact'); 
    
    nexttile; 
    plot(D.t_s,D.rmean_m*1000); 
    ylabel('Position (mm)'); 
    grid on; 
    
    nexttile; 
    plot(D.t_s,D.vmean_mps*1000); 
    hold on; 
    plot(D.t_s,D.vref_mps*1000,'--'); 
    
    ylabel('Velocity (mm/s)'); 
    legend('Interval mean','Logged reference'); 
    grid on; 
    
    nexttile; 
    plot(D.t_s,D.Fmean_N); 
    hold on; 
    plot(D.t_s,mass_kg*D.a_mps2); 
    plot(D.t_s,D.Festimate_N); 
    
    xlabel('Time (s)'); 
    ylabel('Force (N)'); 
    legend('Mean command','m a','Estimated friction'); 
    grid on; 
 
    %% Figure 5: grouped medians
    fig5 = groupedPlot(M,group_names{g},min_samples_per_bin); 
    
    figs = [fig1 fig2 fig3 fig4 fig5]; 
    
    for k = 1:numel(figs) 
        set(figs(k),'Name',...
            [group_names{g} ' | ' get(figs(k),'Name')]); 
        
        figure(figs(k)); 
        sgtitle(strrep(group_names{g},'_',' ')); 
    end 
 
end 
 
%% Functions
function fig = groupedPlot(M,labelName,minCount) 

rEdges = -45:5:45; 
vEdges = [2 10 20 30 40 50 65 85 110 140]; 

speedGroups = [...
    2 15;
    15 30;
    30 50;
    50 140];

positionGroups = [...
    -45 -15;
    -15 15;
    15 45]; 

r = M.rmean_m*1000; 
v = M.vmean_mps*1000; 

resistance = sign(v).*M.Festimate_N; 

fig = figure(...
    'Color','w',...
    'Position',[80 80 1200 780]); 

tiledlayout(2,2,...
    'TileSpacing','compact',...
    'Padding','compact'); 

colors = lines(4); 

for d = 1:2 
    
    direction = 3-2*d; 
    
    if direction == 1
        label = 'Positive motion';
    else
        label = 'Negative motion';
    end 
    
    nexttile(d); 
    hold on; 
    
    for k = 1:size(speedGroups,1) 
        
        q = sign(v)==direction & ...
            abs(v)>=speedGroups(k,1) & ...
            abs(v)<speedGroups(k,2); 
        
        [c,m,n] = binMedian(...
            r(q),resistance(q),rEdges,minCount); 
        
        plot(c,m,'-o',...
            'Color',colors(k,:),...
            'LineWidth',1.8,...
            'MarkerSize',5,...
            'DisplayName',...
            sprintf('%g-%g mm/s',speedGroups(k,:))); 
        
        fprintf('%s, speed %g-%g: %d supported position bins\n',...
            label,speedGroups(k,:),nnz(n>=minCount)); 
    end 
    
    title([label ': position effect']); 
    xlabel('Position r (mm)'); 
    ylabel('Resistance estimate sign(v)F (N)'); 
    
    grid on; 
    box on; 
    legend('Location','best'); 
    xlim([-45 45]); 
    
    nexttile(d+2); 
    hold on; 
    
    for k = 1:size(positionGroups,1) 
        
        q = sign(v)==direction & ...
            r>=positionGroups(k,1) & ...
            r<positionGroups(k,2); 
        
        [c,m,n] = binMedian(...
            abs(v(q)),resistance(q),vEdges,minCount); 
        
        plot(c,m,'-o',...
            'Color',colors(k,:),...
            'LineWidth',1.8,...
            'MarkerSize',5,...
            'DisplayName',...
            sprintf('r: %g to %g mm',positionGroups(k,:))); 
        
        fprintf('%s, position %g to %g: %d supported speed bins\n',...
            label,positionGroups(k,:),nnz(n>=minCount)); 
    end 
    
    title([label ': speed effect']); 
    xlabel('Measured speed |v| (mm/s)'); 
    ylabel('Resistance estimate sign(v)F (N)'); 
    
    grid on; 
    box on; 
    legend('Location','best'); 
    xlim([0 85]); 
end 

axesList = findall(fig,'Type','axes'); 
lims = vertcat(axesList.YLim); 

shared = [...
    min(0,min(lims(:,1))),...
    max(lims(:,2))]; 

set(axesList,'YLim',shared); 

sgtitle(strrep(labelName,'_',' ')); 

end 


function [centers,medians,counts] = binMedian(x,y,edges,minCount) 

centers = (edges(1:end-1)+edges(2:end))/2; 
medians = nan(size(centers)); 
counts = zeros(size(centers)); 

b = discretize(x,edges); 

for j = 1:numel(centers) 
    q = b==j; 
    counts(j) = nnz(q); 
    
    if counts(j) >= minCount
        medians(j) = median(y(q));
    end 
end 

end