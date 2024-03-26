yaw = 0;  
pitch = pi/2; 
roll = pi/2;
dcm = angle2dcm( yaw, pitch, roll)
[theta, phi, psi] = dcm2angle(dcm,"YXZ","ZeroR3")
dcm2 = angle2dcm(theta, phi, psi,"YXZ")
eul = [0.5 pi/2 pi/2];
rotmZYX = eul2rotm(eul)