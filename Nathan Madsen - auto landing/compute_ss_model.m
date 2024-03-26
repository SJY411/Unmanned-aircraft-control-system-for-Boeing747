function [A_lon,B_lon,A_lat,B_lat] = compute_ss_model(filename,x_trim,u_trim)
% x_trim is the trimmed state,
% u_trim is the trimmed input
  
% add stuff here  
[A,B,~,~]=linmod(filename,x_trim,u_trim);
lon_sel=[ 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
          0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
          0, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
lon_selu=[ 1, 0;
           0, 0;
           0, 0;
           0, 1];
lat_sel=[ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
          0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
          0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
          0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0];
lat_selu=[ 0, 0;
           1, 0;
           0, 1;
           0, 0];
A_lon=lon_sel*A*lon_sel';
B_lon=lon_sel*B*lon_selu;
A_lat=lat_sel*A*lat_sel';
B_lat=lat_sel*B*lat_selu;