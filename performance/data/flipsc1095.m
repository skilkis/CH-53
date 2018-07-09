
fid= fopen('SC1095_upper.dat','r');
XYu = fscanf(fid,'%g %g',[2 Inf]) ; 
fclose(fid) ;

fid= fopen('SC1095_lower.dat','r');
XYl = fscanf(fid,'%g %g',[2 Inf]) ; 
fclose(fid) ;

XYu = XYu';
XYl = XYl';

x_CST_u = flipud(XYu(:,1));      %flip so that order is from TE to LE  
y_CST_u = flipud(XYu(:,2));      %flip so that order is from TE to LE

x_CST_l = XYl(2:end,1);     %in correct order to be appended to upper coords
y_CST_l = XYl(2:end,2);     %"", omitting 1st index which is (0,0)

x_CST = [x_CST_u ;
          x_CST_l];         %  Build X coordinate vector
y_CST = [y_CST_u ;
          y_CST_l];         %  Build Y coordinate vector
coord_CST = [x_CST, y_CST]; %  Build Coordinates Matrix

fid = fopen('SC1095.dat','wt');
for i= 1:length(coord_CST)                  %(re)write coordinate file
    fprintf(fid,'%g %g\n',coord_CST(i,1),coord_CST(i,2));
end


