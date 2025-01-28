%% Calling simulink model and security

clc
clear all

close all

global loopActive lengthI roadLength carLength offset blueCar redCar decelR decelLim downClick reactTime
reactTime = -1;
downClick = 0;
%% Setup animation
initPos = -55.8;
decelR = 1.1;
% Animation Setup
roadLength = 100;     % Total length of the road
carLength = 5;        % Length of each car
offset = 10;
loopActive = true;
% Initial plot for the road and cars
figureHandle = figure; 
set(figureHandle, 'WindowKeyPressFcn', @keyPressCallback);
axis([min(roadLength+initPos-offset-carLength,0) roadLength 0 10]); 
hold on;

% Draw the road and cars
plot([min(roadLength+initPos-offset-carLength,0), roadLength], [2, 2], 'k', 'LineWidth', 2);  % Road line
plot([min(roadLength+initPos-offset-carLength,0), roadLength], [8, 8], 'k', 'LineWidth', 2);  % Road line
blueCar = rectangle('Position', [(roadLength+initPos)-carLength-offset, 2, carLength, 1], 'FaceColor', 'blue');
redCar = rectangle('Position', [roadLength-carLength-offset, 2, carLength, 1], 'FaceColor', 'red'); 


%% Conditions for rainy road
Gain = 5000;
InitSpeed = [20 70]; 
decelLim = -100;

[A,B,C,D,Kess, Kr, Ke, uD] = designControl(secureRand(),Gain);

P = [0.6 0.4; 0.85 0.15];

mc = dtmc(P);

numSteps = 1;

numScenarios = 1;

reactionScale = 0.01;

reactionSetupArray = 1;%0.1:0.1:1.6;



for setpoint = 1:size(reactionSetupArray,2)
    
    reactionSetup = reactionSetupArray(setpoint);
    tic
    for scene = 1:numScenarios
        scenario = simulate(mc,numSteps); % 2 is HCW, 1 is LCW
        k = 1;
        collideVal = [];
        collisionStat(scene) = 0;
        switchNum(scene) = 0;
        for i = 1:size(scenario(1),1)
            
            xmin=InitSpeed(1);
            xmax=InitSpeed(2);
            n=1;
            rng('shuffle')
            temp = normrnd(0,0.5,1,n);%generating a standard normal distribution
            initSpeedA=xmin+(xmax-xmin)*abs(temp)
            %initSpeedA = 30
            temp = normrnd(0,0.5,1,n);
            initSpeedB = xmin+(xmax-xmin)*abs(temp)
            %initSpeedB = 25
            doSwitch = 0;
            
            if(scenario(i) == 1)
                %% Actual Human reaction time model
            
                AverageUserMeanHrNormal = 61;
                AverageUserSDHrNormal = 14;
            
                currentHR = normrnd(AverageUserMeanHrNormal,AverageUserSDHrNormal);
            
                AverageUserMeanRRNormal = 17;
                AverageUserSDRRNormal = 8;
            
                currentRR = normrnd(AverageUserMeanRRNormal,AverageUserSDRRNormal);
            
                ActReactionTime = reactionScale*currentHR/currentRR;
                decelLim = -200;
            else
                AverageUserMeanHrNormal = 92;
                AverageUserSDHrNormal = 23;
            
                currentHR = normrnd(AverageUserMeanHrNormal,AverageUserSDHrNormal);
            
                AverageUserMeanRRNormal = 26;
                AverageUserSDRRNormal = 16;
            
                currentRR = normrnd(AverageUserMeanRRNormal,AverageUserSDRRNormal);
            
                ActReactionTime = reactionScale*currentHR/currentRR;
                decelLim = -150;
            end
        
        %for i = min((InitSpeed)):max(InitSpeed)
            
            %% Prediction logic (using the simulator ideal predictions)
            load_system('LaneMaintainSystem3CarProposal.slx')
            set_param('LaneMaintainSystem3CarProposal/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
            set_param('LaneMaintainSystem3CarProposal/VehicleKinematics/vx','InitialCondition',num2str(initSpeedB))
            set_param('LaneMaintainSystem3CarProposal/VehicleKinematics/sx','InitialCondition',num2str(initPos))

            set_param('LaneMaintainSystem3CarProposal/CARA/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
            set_param('LaneMaintainSystem3CarProposal/CARA/VehicleKinematics/vx','InitialCondition',num2str(initSpeedA))
            set_param('LaneMaintainSystem3CarProposal/CARA/VehicleKinematics/sx','InitialCondition',num2str(initPos))
            
            
            simModel = sim('LaneMaintainSystem3CarProposal.slx');

            


            stoppingDistance = simModel.sx1.Data(end);
            stoppingTime = simModel.sx1.Time(end);
            
            vADecel = simModel.ax1.Data;
            vBDistance = simModel.sx1.Data;
            speedDiff = normrnd(10,15,1,1);
            vBDecel = simModel.axB.Data;
            for kt = 1:size(vADecel,1)
                yB = membershipFunctionBrakingNA(abs(vADecel(kt)));
                yD = membershipFunctionDistance(vBDistance(kt));
                yR = membershipFunctionRoad(speedDiff);

                %% Fuzzy rules
                % if A decel H & Far & N road --> M
                % if A decel H & Close & N road --> H
                % if A decel H & Far & P road --> L
                % if A decel H & Close & P road --> M

                % if A decel M & Far & N road --> M
                % if A decel M & Close & N road --> H
                % if A decel M & Far & P road --> M
                % if A decel M & Close & P road --> L

                % if A decel L & Far & N road --> M
                % if A decel L & Close & N road --> M
                % if A decel L & Far & P road --> L
                % if A decel L & Close & P road --> L
                
                % Low brake membership    
                
                lYB = max([min([yB(3) yD(1) yR(2)]),min([yB(2) yD(2) yR(2)]),min([yB(1) yD(1) yR(2)]),min([yB(1) yD(2) yR(2)])]);

                % Medium brake membership    
                
                mYB = max([min([yB(3) yD(2) yR(1)]),min([yB(3) yD(1) yR(2)]),min([yB(2) yD(2) yR(1)]),min([yB(2) yD(2) yR(2)]),min([yB(1) yD(2) yR(1)]),min([yB(1) yD(1) yR(1)])]);

                % High brake membership    
                
                hYB = max([min([yB(3) yD(2) yR(1)]),min([yB(2) yD(2) yR(1)])]);

                [lYB mYB hYB]/sum([lYB mYB hYB])

                decelB(kt) = -(defuzzBrake([lYB mYB hYB]/sum([lYB mYB hYB])));

                if(abs(decelB(kt)) > 0.75*abs(decelLim))
                    doSwitch = 1;
                    break;
                    
                end


                %%
                
            end

            
            doSwitch = 1;
    
            %% Actually what happens?
            if(doSwitch)
                disp('Switch to Human')
                load_system('Level3Model.slx')
            
                set_param('Level3Model/VehicleKinematics/Saturation','LowerLimit',num2str(100*decelLim))
                set_param('Level3Model/VehicleKinematics/vx','InitialCondition',num2str(initSpeedB))
                set_param('Level3Model/VehicleKinematics/sx','InitialCondition',num2str(initPos))
                set_param('Level3Model/CARA/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
                set_param('Level3Model/CARA/VehicleKinematics/vx','InitialCondition',num2str(initSpeedA))
                set_param('Level3Model/CARA/VehicleKinematics/sx','InitialCondition',num2str(initPos))
                
                set_param('Level3Model/Constant1','Value',num2str(simModel.ax1.Time(kt)))

                set_param('Level3Model/Step','Time',num2str(ActReactionTime+simModel.ax1.Time(kt)))
                set_param('Level3Model/Step','After',num2str(1.1*decelLim))
                %samplingFrequency = 10;
                %set_param('Level3Model', 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(1/samplingFrequency));
                set_param('Level3Model', 'StopTime', num2str(10));
            
                outHuman = sim('Level3Model.slx');
                
                
                totTime = outHuman.tout;
                minTime = 0.1;
                lengthI = 0;
                for lp = 1:max(size(totTime))
                    if(totTime(lp)-lengthI(end) > minTime)
                        lengthI = [lengthI totTime(lp)];
                    end
                end

                if(totTime(end) ~= lengthI(end))
                    lengthI = [lengthI totTime(end)];
                end

                loopActive = true;

%                 for pk = 1:1:max(size(lengthI))-1
%                     stopTime = lengthI(pk+1); % For example, 10 seconds
%                     set_param('Level3Model', 'StopTime', num2str(stopTime));
% 
%                     outHuman = sim('Level3Model.slx');
% 
%                     posB = roadLength + outHuman.sx1.Data(end)-carLength-offset;
%                     posR = roadLength-carLength-offset;
%                     updateAnimation(posB, posR, blueCar, redCar, 'Level3Model', carLength)
%                     if(posB > posR)
%                         disp('collision')
%                         break
%                     end
%                     
%                 end

                if(outHuman.sx1.Data(end) >= 0)
                    collideVal(k) = 1;
                    break;
    
                else
                    collideVal(k) = 0;
                end
    
            else
                load_system('LaneMaintainSystem3CarProposal.slx')
                set_param('LaneMaintainSystem3CarProposal/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
                set_param('LaneMaintainSystem3CarProposal/VehicleKinematics/vx','InitialCondition',num2str(initSpeedB))
                set_param('LaneMaintainSystem3CarProposal/VehicleKinematics/sx','InitialCondition',num2str(initPos))

                set_param('LaneMaintainSystem3CarProposal/CARA/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
                set_param('LaneMaintainSystem3CarProposal/CARA/VehicleKinematics/vx','InitialCondition',num2str(initSpeedA))
                set_param('LaneMaintainSystem3CarProposal/CARA/VehicleKinematics/sx','InitialCondition',num2str(initPos))
            
            
                simModel = sim('LaneMaintainSystem3CarProposal.slx');

                totTime = simModel.tout;
                minTime = 0.1;
                lengthI = 0;
                for lp = 1:max(size(totTime))
                    if(totTime(lp)-lengthI(end) > minTime)
                        lengthI = [lengthI totTime(lp)];
                    end
                end

                if(totTime(end) ~= lengthI(end))
                    lengthI = [lengthI totTime(end)];
                end

                loopActive = false;

%                 for pk = 1:1:max(size(lengthI))-1
%                     stopTime = lengthI(pk+1); % For example, 10 seconds
%                     set_param('LaneMaintainSystem3CarProposal', 'StopTime', num2str(stopTime));
% 
%                     outHuman = sim('LaneMaintainSystem3CarProposal.slx');
% 
%                     posB = roadLength + outHuman.sx1.Data(end)-carLength-offset;
%                     posR = roadLength-carLength-offset;
%                     updateAnimation(posB, posR, blueCar, redCar, 'Level3Model', carLength)
%                     if(posB > posR)
%                         disp('collision')
%                         break
%                     end
%                     
%                 end
                

                
    
                if(simModel.sx1.Data(end) >= 0)
                    collideVal(k) = 1;
                    break;
    
                else
                    collideVal(k) = 0;
                end
                
    
            end
            k = k + 1;
    
            
           
        end
        %if(collideVal(k-1) == 1)
        collisionStat(scene) = k-1;
       
    
    %end
    end
    
    
    
    [G,H] = find(collisionStat > numSteps);
    
    collisionStat(H) = 0;
    
    [G1,H1] = find(collisionStat > 0);
    
    numCollisions(setpoint) = size(H1,2);
    numSwitches(setpoint) = sum(switchNum);
    toc
end


function updateAnimation(posB, posR, blueCar, redCar, modelName, carLength)
    % Retrieve data from Simulink model output
    posBlueCar = posB;
    posRedCar = posR;

    % Update blue and red car positions in the animation
    set(blueCar, 'Position', [posBlueCar, 2, carLength, 1]);
    set(redCar, 'Position', [posRedCar, 2, carLength, 1]);

    % Update display title with simulation time
    simTime = get_param(modelName, 'SimulationTime');
    title(sprintf('Simulation Time: %.2f s', simTime));

    % Refresh figure to reflect updated positions
    drawnow;
end



function startAnimationHuman(lengthI,roadLength,carLength,offset,blueCar,redCar)
global loopActive reactTime
    disp('Human')
    for pk = 1:1:max(size(lengthI))-1
        stopTime = lengthI(pk+1); % For example, 10 seconds
        set_param('Level3Model', 'StopTime', num2str(stopTime));

        outHuman = sim('Level3Model.slx');

        posB = roadLength + outHuman.sx1.Data(end)-carLength-offset;
        posR = roadLength-carLength-offset;
        distThresh = 15;
        %posR-posB
        if(posR-posB < distThresh)
            beep;
            set(gcf, 'Color', [1 1 0]);
            tic
        end
        if(outHuman.vx1.Data(end)<0)
            loopActive = false;
            break
        end
        updateAnimation(posB, posR, blueCar, redCar, 'Level3Model', carLength)
        if(posB > posR)
            disp('collision')
            break
        

        end
        
    end


    fileName = 'react.csv';
    if isfile(fileName)
        data = csvread(fileName);
    else
        data = []; % Initialize an empty array if file doesn't exist
    end
    
    % Append the new number to the data
    data = [data; reactTime];
    
    % Write the updated data back to the CSV file
    csvwrite(fileName, data);

    AnimationOutreachKeyProposal
end


function startAnimationAuto(lengthI,roadLength,carLength,offset,blueCar,redCar)

    for pk = 1:1:max(size(lengthI))-1
        stopTime = lengthI(pk+1); % For example, 10 seconds
        set_param('LaneMaintainSystem3CarProposal', 'StopTime', num2str(stopTime));

        outHuman = sim('LaneMaintainSystem3CarProposal.slx');

        posB = roadLength + outHuman.sx1.Data(end)-carLength-offset;
        posR = roadLength-carLength-offset;
        updateAnimation(posB, posR, blueCar, redCar, 'Level3Model', carLength)
        distThresh = 15;
        %posR-posB
        if(posR-posB < distThresh)
            beep;
            tic
        end

        if(posB > posR)
            disp('collision')
            break
        end
        
    end
end

function keyPressCallback(~, event)
    global loopActive lengthI roadLength carLength offset blueCar redCar decelR decelLim downClick reactTime
    if strcmp(event.Key, 'uparrow')
        disp('Up arrow pressed - starting animation loop');
        loopActive
        if(loopActive)   % Set the flag to start the loop
            startAnimationHuman(lengthI,roadLength,carLength,offset,blueCar,redCar);
        else
            startAnimationAuto(lengthI,roadLength,carLength,offset,blueCar,redCar);
        end
    elseif strcmp(event.Key, 'downarrow')

        if(loopActive)
            perC = 70;
            if(downClick == 0)
                reactTime = toc
                downClick = 1;
            else
                
                
            end
            decelR = (1+perC/100)*decelR
            set_param('Level3Model/Step','Time',num2str(0))
            set_param('Level3Model/Constant1','Value',num2str(0))
            set_param('Level3Model/Step','After',num2str(decelR*decelLim))
        else


        end
    end

end



%save('DataForScenarios.mat','collideVal','IODataForSurrogate')

