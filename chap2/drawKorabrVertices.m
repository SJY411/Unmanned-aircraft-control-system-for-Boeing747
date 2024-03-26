function drawKorabrVertices(uu)
%使用点面模型制作太空船动画
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    phi      = uu(4);       % roll angle         
    theta    = uu(5);       % pitch angle     
    psi      = uu(6);       % yaw angle     
    t        = uu(7);       % time

    % define persistent variables 
    persistent spacecraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        [Vertices, Faces, facecolors] = korabrVFC;
        spacecraft_handle = drawSpacecraftBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Spacecraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-10,10,-10,10,-10,10]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawSpacecraftBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           spacecraft_handle);
    end
end

function handle = drawSpacecraftBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V', phi, theta, psi)';  % rotate spacecraft
  V = translateKorabr(V', pn, pe, pd)';  % translate spacecraft
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

function XYZ = translateKorabr(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

function [V,F,colors] = korabrVFC()
    % 定义顶点
    V = [...
        1    1    0;... % point 1
        1   -1    0;... % point 2
        -1   -1    0;... % point 3
        -1    1    0;... % point 4
        1    1   -2;... % point 5
        1   -1   -2;... % point 6
        -1   -1   -2;... % point 7
        -1    1   -2;... % point 8
        1.5  1.5  0;... % point 9
        1.5 -1.5  0;... % point 10
        -1.5 -1.5  0;... % point 11
        -1.5  1.5  0;... % point 12
    ];

    % define faces as a list of vertices numbered above
    F = [...
        1, 2,  6,  5;...  % front
        4, 3,  7,  8;...  % back
        1, 5,  8,  4;...  % right 
        2, 6,  7,  3;...  % left
        5, 6,  7,  8;...  % top
        9, 10, 11, 12;... % bottom
        ];

    % define colors for each face    
    myred = [1, 0, 0];
    mygreen = [0, 1, 0];
    myblue = [0, 0, 1];
    myyellow = [1, 1, 0];
    mycyan = [0, 1, 1];

    colors = [...
        myred;...    % front
        mygreen;...  % back
        myblue;...   % right
        myyellow;... % left
        mycyan;...   % top
        mycyan;...   % bottom
        ];
end