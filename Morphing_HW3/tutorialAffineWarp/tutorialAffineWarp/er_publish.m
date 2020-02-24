function er_publish( FileName )
%% 
% FileName: Name of the .m file for the tutorial, e.g., tutorial_probdist.m
%



opts.stylesheet='gpeyre.xsl';
opts.format='html';
publish(FileName,opts);

% Copy style file to html directory
copyfile( 'style.css', 'html/.' );

return 
