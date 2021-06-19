function moveL(clientID, target ,pos,speed)
sim=remApi('remoteApi');
%get target position and orientation
[r,p]=sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_blocking);
[r,o]=sim.simxGetObjectOrientation(clientID, target, -1, sim.simx_opmode_blocking);
%To get to the desired orientation we have two possibilities (the positive
%or negative direction. So, we need to add these lines to calculate the
%shortest one:
for i=1:3
   if ((abs(pos(i+3)-o(i))>pi)&&(o(i)<0))
       o(i)=o(i)+2*pi;
   elseif ((abs(pos(i+3)-o(i))>pi)&&(o(i)>0))
       o(i)=o(i)-2*pi;
   end
end
old_pos=[p o];
delta_pos= pos - old_pos;
distance=norm(delta_pos);
samples_number=round(distance*50);
for i=1:samples_number;
    intermidiate_pos= old_pos + (delta_pos/samples_number);
    % pause
    tic;
    while(toc < (distance/(speed*samples_number)))
    end
    % set the intermediate_pos
    sim.simxSetObjectPosition(clientID, target, -1, intermidiate_pos,sim.simx_opmode_blocking);
    sim.simxSetObjectOrientation(clientID, target, -1, intermidiate_pos(4:6),sim.simx_opmode_blocking);
    old_pos=intermidiate_pos;
end 