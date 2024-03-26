function drawAircraft_quaternion(uu)
    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
%     u        = uu(4);       
%     v        = uu(5);       
%     w        = uu(6);       
%     e0       = uu(7);
%     e1       = uu(8);
%     e2       = uu(9);
%     e3       = uu(10);
    quat     = uu(7:10)';
%     p        = uu(11);       % roll rate
%     q        = uu(12);       % pitch rate     
%     r        = uu(13);       % yaw rate    
    t        = uu(14);       % time

    % define persistent variables 
    persistent spacecraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        [Vertices, Faces, facecolors] = defineSpacecraftBody;
        spacecraft_handle = drawAircraftBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,quat,...
                                               [],'normal');
        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-1000,1000,-1000,1000,-1000,1000]);
        grid on
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawAircraftBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,quat,...
                           spacecraft_handle);
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawAircraftBody(V,F,patchcolors,...
                                     pn,pe,pd,quat,...
                                     handle,mode)
  V = rotate(V', quat)';  % rotate spacecraft
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
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,quat)
  % define rotation matrix
  R = Quaternion2Rotation(quat);
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

