function drawAircraft(uu)
    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    persistent spacecraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    persistent Waypoints  % 用于存储飞行路点
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        Waypoints = [pe, pn, -pd];
        [Vertices, Faces, facecolors] = defineSpacecraftBody;
%         track_handle = drawTrack(Waypoints,[]);
        spacecraft_handle = drawAircraftBody(Vertices,Faces,facecolors,Waypoints,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-3000,3000,-3000,3000,0,2000]);
        grid on
        hold on
        
    % at every other time step, redraw base and rod
    else
        Waypoints = [Waypoints; pe, pn, -pd];
        drawAircraftBody(Vertices,Faces,facecolors,Waypoints,...
                           pn,pe,pd,phi,theta,psi,...
                           spacecraft_handle);
%         drawTrack(Waypoints, track_handle);
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawAircraftBody(V,F,patchcolors,WP,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V', phi, theta, psi)';  % rotate spacecraft
  V = translate(V', pn, pe, pd)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  if isempty(handle)
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  hold on
  figure(1), line(WP(:,1), WP(:,2), WP(:,3), 'color', 'red', 'linewidth', 1.5);
  hold off
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
    hold on
    figure(1), line(WP(:,1), WP(:,2), WP(:,3), 'color', 'red', 'linewidth', 1.5);
    hold off
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;
  % rotate vertices
  XYZ = R*XYZ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define spacecraft vertices and faces
function [V,F,colors] = defineSpacecraftBody()
    % Define the vertices (physical location of vertices
    V = [...
        6 0 0;
        3.5 1.5 -2.1;
        3.5 -1.5 -2.1;
        3.5 -1.5 2.1;
        3.5 1.5 2.1;
        -14.6 0 0;
        0 10 0;
        -6 10 0;
        -6 -10 0;
        0 -10 0;
        -11 5 0;
        -14.6 5 0;
        -14.6 -5 0;
        -11 -5 0;
        -12 0 0;
        -14.6 0 -4.8;
    ]*20;

    % define faces as a list of vertices numbered above
    F = [
        1 2 3 nan;
        1 3 4 nan;
        1 4 5 nan;
        1 5 2 nan;%机头
        6 2 3 nan;
        6 3 4 nan;
        6 4 5 nan;
        6 5 2 nan;%机身
        6 15 16 nan;%方向舵
        7 8 9 10;%机翼
        11 12 13 14;%升降舵
        ];

    % define colors for each face    
    myred = [1, 0, 0];
    mygreen = [0, 1, 0];
    myblue = [0, 0, 1];
    myyellow = [1, 1, 0];
    mycyan = [0, 1, 1];

    colors = [...
        myyellow;
        myyellow;
        myyellow;
        myyellow;...  % 机头
        myblue;
        myblue;
        myred;
        myblue;...   % 机身
        mycyan;... % 方向舵
        mygreen;...   % 机翼
        mygreen;...   % 升降舵
        ];
end
