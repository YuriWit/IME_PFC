function hanno = helperAnnotateMicroDopplerSpectrogram(fh)
% This function is only in support of MicroDopplerExample. It may be
% removed in a future release.

hanno(1) = annotation(fh,'textarrow',[0.2 0.13],[0.75 0.63],'String','Blade no. 1',...
    'LineWidth',1,'FontWeight','bold','Color','w','TextColor','w');
hanno(2) = annotation(fh,'textarrow',[0.25 0.2],[0.7 0.63],'String','Blade no. 2',...
    'LineWidth',1,'FontWeight','bold','Color','w','TextColor','w');
hanno(3) = annotation(fh,'textarrow',[0.2 0.13],[0.18 0.3],'String','Blade no. 3',...
    'LineWidth',1,'FontWeight','bold','Color','w','TextColor','w');
hanno(4) = annotation(fh,'textarrow',[0.25 0.2],[0.23 0.3],'String','Blade no. 4',...
    'LineWidth',1,'FontWeight','bold','Color','w','TextColor','w');
hanno(5) = annotation(fh,'doublearrow',[0.12 0.465],[0.45 0.45],'LineWidth',1,'Color','w');
hanno(6) = annotation(fh,'textbox',[0.31 0.35 0.1 0.1],'Color','w','String','Tr',...
    'FontWeight','bold','EdgeColor','none');
hanno(7) = annotation(fh,'doublearrow',[0.291 0.291],[0.46 0.62],'LineWidth',1,'Color','w');
hanno(8) = annotation(fh,'textbox',[0.292 0.55 0.1 0.05],'Color','w','String','Vt',...
    'FontWeight','bold','EdgeColor','none');


