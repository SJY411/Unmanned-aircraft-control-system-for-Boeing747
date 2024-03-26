fuse_l1=1.5; % center of mass to tip of fuselage
fuse_l2=1; % center of mass to widest part of fuselage
fuse_l3=5; % center of mass to back of fuselage
fuse_h=1;
fuse_w=1;
wing_l=2; % length of wing along fuselage
wing_w=6; % wingspan
tail_h=1;
tailwing_l=1;
tailwing_w=3;

% wheel_rad=.1;
% theta=(0:2*pi/10:2*pi)';
% circle=wheel_rad*[cos(theta),zeros(size(theta)),sin(theta)];
% Define the vertices (physical location of vertices
  V = .5*[...
    fuse_l1,        0,              0;... % point 1
    fuse_l2,        fuse_w/2,       -fuse_h/2;... % point 2
    fuse_l2,        -fuse_w/2,      -fuse_h/2;... % point 3
    fuse_l2,        -fuse_w/2,      fuse_h/2;... % point 4
	fuse_l2,        fuse_w/2,       fuse_h/2;... % point 5
    -fuse_l3,       0,              0;... % point 6
    0,              wing_w/2,       0;... % point 7
    -wing_l,        wing_w/2,       0;... % point 8
	-wing_l,        -wing_w/2,      0;... % point 9
    0,              -wing_w/2,      0;... % point 10
    -fuse_l3+tailwing_l,tailwing_w/2,0;... % point 11
    -fuse_l3,       tailwing_w/2,   0;... % point 12
    -fuse_l3,       -tailwing_w/2,  0;... % point 13
    -fuse_l3+tailwing_l,-tailwing_w/2,0;... % point 14
    -fuse_l3+tailwing_l,0,          0;... % point 15
    -fuse_l3,       0,              -tail_h;... % point 16
  ];

% define faces as a list of vertices numbered above
  F = [...
         1,  2,  3,  3;...  % front top
         1,  3,  4,  4;...  % front left
         1,  4,  5,  5;...  % front bottom
         1,  2,  5,  5;...  % front right 
         2,  3,  6,  6;...  % main top 
         3,  4,  6,  6;...  % main left
         4,  5,  6,  6;...  % main bottom 
         2,  5,  6,  6;...  % main right
         7,  8,  9, 10;...  % wings
        11, 12, 13, 14;...  % tailwing
         6, 15, 16, 16;...  % tailfin
        ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

  patchcolors = [...
    mygreen;...  % front top
    mycyan;...  % front left
    myblue;...  % front bottom
    mycyan;...  % front right 
    mygreen;...  % main top 
    mycyan;...  % main left
    myblue;...  % main bottom 
    mycyan;...  % main right
    myred;...  % wings
    myred;...  % tailwing
    myyellow;...  % tailfin
    ];