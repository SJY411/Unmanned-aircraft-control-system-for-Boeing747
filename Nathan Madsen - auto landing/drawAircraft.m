function drawAircraft(uu,V,F,patchcolors)

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
    defl1    = uu(13);       % roll rate
    defl2    = uu(14);       % pitch rate     
    defl3    = uu(15);       % yaw rate   
    t        = uu(16);       % time

    % define persistent variables 
    persistent plane_handle1;
    persistent plane_handle2;
    persistent plane_handle3;
    persistent ax_handle;
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        plane_handle1 = drawPlaneBody(V,F,patchcolors,...
                                               pn,pe,pd,phi,theta,psi,defl1,defl2,defl3,...
                                               [],'normal');
                                           
        V = [1000, 0, 0;
            -1000, 0, 0;
            -1000, 0, 0;
             1000, 0, 0];
        drawGround(V,[],'normal');
        title('Plane Landing')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(180,0)  % set the view angle for figure
        axis([-5,5,-5,200,-2,20]);
        %axis([-1,3,-5,200,-1,2]);
        hold on
        
        
        figure(10), clf
        plane_handle2 = drawPlaneBody(V,F,patchcolors,...
                                               pn,pe,pd,phi,theta,psi,defl1,defl2,defl3,...
                                               [],'normal');
        V = [0, 1000, 0;
             0,-1000, 0;
             0,-1000, 0;
             0, 1000, 0];
        drawGround(V,[],'normal');
        title('Plane Landing')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(90,0)  % set the view angle for figure
        axis([-10,10,0,150,-2,20]);
        hold on
        
        figure(11); clf
        plane_handle3 = drawPlaneBody(V,F,patchcolors,...
                                               pn,pe,pd,phi,theta,psi,defl1,defl2,defl3,...
                                               [],'normal');
        ax_handle=gca;        
        V = [0, 1000, 0;
             0,-1000, 0;
             0,-1000, 0;
             0, 1000, 0];
        drawGround(V,[],'normal');
        title('Plane Landing')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(90,0)  % set the view angle for figure
        axis([pn-5,pn+5,pe-5,pe+5,pd-5,pd+5]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawPlaneBody(V,F,patchcolors,...
                           pn,pe,pd,phi,theta,psi,defl1,defl2,defl3,...
                           plane_handle1);
        drawPlaneBody(V,F,patchcolors,...
                           pn,pe,pd,phi,theta,psi,defl1,defl2,defl3,...
                           plane_handle2);
        drawPlaneBody(V,F,patchcolors,...
                           pn,pe,pd,phi,theta,psi,defl1,defl2,defl3,...
                           plane_handle3);
        set(ax_handle,'xlim',[pe-5,pe+5],'ylim',[pn-5,pn+5],'zlim',[-pd-5,-pd+5]);
    end
end

  
function handle = drawGround(V,handle,mode)
%   V = [-100, 100, 0;
%        -100,-100, 0;
%         100,-100, 0;
%         100, 100, 0];
%   V = [-1000, 0, 0;
%        -1000, 0, 0;
%         1000, 0, 0;
%         1000, 0, 0];
  F=[1,2,3,4];
  mygreen = [0, 1, 0];
  patchcolors=[mygreen];
  
  if isempty(handle),
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
  end

function handle = drawPlaneBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,defl1,defl2,defl3,...
                                     handle,mode)
  lv=size(V,1);
  wrad=.1;
  wheel=[wrad,0, wrad;
         wrad,0,-wrad;
        -wrad,0,-wrad;
        -wrad,0, wrad];
  trans1=ones(4,1)*[.5,   0,  .7-wrad-defl1];
  trans2=ones(4,1)*[-1, 0.5, .45-wrad-defl2];
  trans3=ones(4,1)*[-1,-0.5, .45-wrad-defl3];
  
  F=[F;lv+(1:4);lv+4+(1:4);lv+8+(1:4)];
  patchcolors=[patchcolors; 1 0 0; 1 0 0; 1 0 0];
  
  V=[V;trans1+wheel;trans2+wheel;trans3+wheel];
  V = rotate(V', phi, theta, psi)';  % rotate spacecraft
  V = translate(V', pn, pe, pd)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  if isempty(handle),
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
function XYZ=rotate(XYZ,phi,theta,psi);
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
  R = R_yaw*R_pitch*R_roll;
  % rotate vertices
  XYZ = R*XYZ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

  