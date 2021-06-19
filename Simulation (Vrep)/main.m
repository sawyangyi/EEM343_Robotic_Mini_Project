clear all
close all
clc

sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID >-1)
    disp ('connected to remote API server');  
    
    %object handles
    
   [res,left_target]=sim.simxGetObjectHandle(clientID,'left_target',sim.simx_opmode_blocking); %left arm
   [res,right_target]=sim.simxGetObjectHandle(clientID,'right_target',sim.simx_opmode_blocking); %right arm
 
   
   
   
   %let's define now the target positions needed
    
    fposition3=[0.02159,1.0581,1.5923,0,0,0]; % new position
    fposition4=[0.24659,0.65806,1.5173,0,0,0]; % ori position
    
    fposition5=[-0.26326,1.0824,0.7717,0,0,0]; % new position
    fposition6=[-0.90726,0.82237,1.0467,0,0,0]; % ori position


    
    moveL(clientID, left_target ,fposition3,2); 
    
    pause(2.5);
    
    moveL(clientID, left_target ,fposition4,2); 
    
    pause(2.5);
    

    moveL(clientID, right_target ,fposition5,2); 
    
    pause(2.5);
    
    joint_pos=[28*pi/180,-18*pi/180]
    
    %joint handle
    h=[0,0]
    [r,h(1)]=sim.simxGetObjectHandle(clientID,'LCG',sim.simx_opmode_blocking)
    [r,h(2)]=sim.simxGetObjectHandle(clientID,'RGC',sim.simx_opmode_blocking)
    
    while true
    
    for i=1:2    
    sim.simxSetJointTargetPosition(clientID,h(i),joint_pos(i),sim.simx_opmode_streaming)
    end
    moveL(clientID, right_target ,fposition6,2);
    end
    
 
sim.delete(); % call the destructor 
disp('program ended');
end     