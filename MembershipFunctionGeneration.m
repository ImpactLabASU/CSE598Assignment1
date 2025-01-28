%% Calling simulink model and security

clc
clear all

close all

%% Conditions for rainy road
Gain = 500;
InitSpeed = [20 60]; 
decelLim = -100;

[A,B,C,D,Kess, Kr, Ke, uD] = designControl(secureRand(),Gain);

P = [0.6 0.4; 0.85 0.15];

mc = dtmc(P);

numSteps = 1;

numScenarios = 10;

reactionScale = 0.01;

reactionSetupArray = 1;%0.1:0.1:1.6;


decelLim = -150;    
            
            k = 1;
            speeds = min((InitSpeed)):max(InitSpeed);
            
for i = 1:41
            tic
            %% Prediction logic (using the simulator ideal predictions)
            load_system('LaneMaintainSystem.slx')
            set_param('LaneMaintainSystem/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
            set_param('LaneMaintainSystem/VehicleKinematics/vx','InitialCondition',num2str(speeds(i)))
            
            
            simModel = sim('LaneMaintainSystem.slx');
            
            stoppingDistance = simModel.sx1.Data(end);
            stoppingTime = simModel.sx1.Time(end);
            decelMax(i) = max(abs(simModel.ax1.Data));
             
        
            %IODataForSurrogate{k} =  actualModel;
            
            close_system('LaneMaintainSystem.slx',0)
            
    
    
            
            toc
end
%save('DataForScenarios.mat','collideVal','IODataForSurrogate')