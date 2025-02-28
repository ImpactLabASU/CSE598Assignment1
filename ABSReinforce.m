%% Calling simulink model and security

clc
clear all

close all

global loopActive lengthI roadLength carLength offset blueCar redCar decelR decelLim downClick reactTime
reactTime = -1;
downClick = 0;
%% Setup animation



%% Conditions for rainy road
Gain = 5000;
InitSpeed = [20 70]; 
decelLim = -100;

[A,B,C,D,Kess, Kr, Ke, uD] = designControl(secureRand(),Gain);

P = [0.6 0.4; 0.85 0.15];

mc = dtmc(P);

numSteps = 1;

numScenarios = 250;

reactionScale = 0.01;

reactionSetupArray = 1;%0.1:0.1:1.6;

%% Input reward structure

matrix = getUserInputArray();
epA = getUserChoice();
[hU, meanRT, stdRT] = getHumanChoice();


%%

DOPLOT = 0;

stateSet = [];
ActionSet = [];
RewardSet = [];
RewardA = [];
RewardH = [];
initNum = 10;
%epsilon = [1*ones(1,10) 0.8*ones(1,20) 0.7*ones(1,30) 0.6*ones(1,20) 0.5*ones(1,10) 0.4*ones(1,10) 0.4*ones(1,10) 1*ones(1,10) 0.4*ones(1,20) 1*ones(1,30) 0.4*ones(1,20) 0.4*ones(1,10) 0*ones(1,50)];
epsilon = [ones(1,initNum) 0.2*ones(1,numScenarios-initNum)];
winCount = [0,0];
lossCount = [0,0];
actWinC = [0,0];
actLossC = [0,0];
alpha = 0.2;
dangerD = [-35 -50];
dangerV = [35 10];

actWinCSet = zeros(3,2);
actLossSet = zeros(3,2);

for setpoint = 1:numScenarios
    initPosV = [-55.8 -23.8];
    decelR = 1.1;
    % Animation Setup
    roadLength = 100;     % Total length of the road
    carLength = 5;        % Length of each car
    offset = 10;
    loopActive = true;
    % Initial plot for the road and cars
    if(DOPLOT)
        figureHandle = figure; 
        set(figureHandle, 'WindowKeyPressFcn', @keyPressCallback);
        axis([min(roadLength+initPos-offset-carLength,0) roadLength 0 10]); 
        hold on;
        
        % Draw the road and cars
        plot([min(roadLength+initPos-offset-carLength,0), roadLength], [2, 2], 'k', 'LineWidth', 2);  % Road line
        plot([min(roadLength+initPos-offset-carLength,0), roadLength], [8, 8], 'k', 'LineWidth', 2);  % Road line
        blueCar = rectangle('Position', [(roadLength+initPos)-carLength-offset, 2, carLength, 1], 'FaceColor', 'blue');
        redCar = rectangle('Position', [roadLength-carLength-offset, 2, carLength, 1], 'FaceColor', 'red'); 
    end
    
    
    
    %reactionSetup = reactionSetupArray(setpoint);
    %tic
    %for scene = 1:numScenarios
        %scenario = simulate(mc,numSteps); % 2 is HCW, 1 is LCW
        k = 1;
        collideVal = [];

        %for i = 1:size(scenario(1),1)
            
            xmin=InitSpeed(1);
            xmax=InitSpeed(2);
            dmin = initPosV(1);
            dmax = initPosV(2);
            n=1;
            rng('shuffle')
            temp = normrnd(0,0.5,1,n);%generating a standard normal distribution
            initSpeedA=xmin+(xmax-xmin)*abs(temp);
            %initSpeedA = 30
            temp = normrnd(0,0.5,1,n);
            initSpeedB = xmin+(xmax-xmin)*abs(temp);
            temp = normrnd(0,0.5,1,n);
            initPos = dmin + (dmax-dmin)*abs(temp);

            
            
            
            %%

            %initSpeedB = 25
            doSwitch = 0;
            b = hU;
            if(b == 1)
                %% Actual Human reaction time model
            
                AverageUserMeanHrNormal = 61;
                AverageUserSDHrNormal = 14;
            
                currentHR = normrnd(AverageUserMeanHrNormal,AverageUserSDHrNormal);
            
                AverageUserMeanRRNormal = 17;
                AverageUserSDRRNormal = 8;
            
                currentRR = normrnd(AverageUserMeanRRNormal,AverageUserSDRRNormal);
            
                ActReactionTime = reactionScale*currentHR/currentRR;
                decelLim = -150;
            elseif(b == 0)
                AverageUserMeanHrNormal = 92;
                AverageUserSDHrNormal = 23;
            
                currentHR = normrnd(AverageUserMeanHrNormal,AverageUserSDHrNormal);
            
                AverageUserMeanRRNormal = 26;
                AverageUserSDRRNormal = 16;
            
                currentRR = normrnd(AverageUserMeanRRNormal,AverageUserSDRRNormal);
            
                ActReactionTime = reactionScale*currentHR/currentRR;
                decelLim = -110;
            elseif(b == 2)
                ActReactionTime = normrnd(meanRT,stdRT);
            end
        
        %for i = min((InitSpeed)):max(InitSpeed)
            
            %% Empirical Reward Computation

            % Reward for Autonomous action
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
            stateSet = [stateSet; [initPos, initSpeedB-initSpeedA]];
            if(stoppingDistance < 0.1)
                RewardA = [RewardA matrix(2,1)];
                lossCount(1) =  1; 
            else
                RewardA = [RewardA matrix(2,2)];
                winCount(1) =  1;
            end
    
            
            %Reward for human action
                disp('Switch to Human')
                load_system('Level3Model.slx')
                kt = 5;
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
                
                if(outHuman.sx1.Data(end) < 0.1)
                    RewardH = [RewardH matrix(1,1)];
                    lossCount(2) = 1;
                else
                    RewardH = [RewardH matrix(1,2)];
                    winCount(2) =  1;
                end
                
        %% Epsilon greedy algorithm
        randP = rand();
        if(randP > epsilon(setpoint))
            
            % find autonomous average reward

            [G,H] = find(ActionSet == 1);
            if(epA == 0)
                meanAR = mean(RewardSet(H)); % epsilon greedy only
            elseif(epA == 1)
                meanAR = (1-alpha)*mean(RewardSet(H)) + alpha*RewardSet(H(end)); %  non-stationary    
            elseif(epA == 2) %% Optimal bayesian approach
                meanAR = (actWinC(1) + 1)/(actWinC(1)+actLossC(1)+2);
            elseif(epA == 3) %% Context + Optimal
                


            end
            

            % find human average reward
            [G,H] = find(ActionSet == 0);
            if(epA == 0)
                meanHR = mean(RewardSet(H)); % epsilon greedy only
            elseif(epA == 1)
                meanHR = (1-alpha)*mean(RewardSet(H)) + alpha*RewardSet(H(end)); %  non-stationary    
            elseif(epA == 2) %% Optimal bayesian approach
                meanHR = (actWinC(2) + 1)/(actWinC(2)+actLossC(2)+2);
            end

            if(meanAR > meanHR)
                
                ActionSet = [ActionSet 1]; % 1 is autonomous action
                RewardSet = [RewardSet RewardA(end)];
                if(winCount(1) == 1)
                    actWinC(1) = actWinC(1) + 1;
                else
                    actLossC(1) = actLossC(1) + 1;
                end


            else

                ActionSet = [ActionSet 0]; % 0 is human action
                RewardSet = [RewardSet RewardH(end)];
                if(winCount(2) == 1)
                    actWinC(2) = actWinC(2) + 1;
                else
                    actLossC(2) = actLossC(2) + 1;
                end

            end

            lossCount = [0,0];
            winCount = [0,0];

        else

            if(rand() > 0.5)
                ActionSet = [ActionSet 0]; % 0 is human action
                RewardSet = [RewardSet RewardH(end)];

                if(winCount(2) == 1)
                    actWinC(2) = actWinC(2) + 1;
                else
                    actLossC(2) = actLossC(2) + 1;
                end

            else
                ActionSet = [ActionSet 1]; % 1 is autonomous action
                RewardSet = [RewardSet RewardA(end)];

                if(winCount(1) == 1)
                    actWinC(1) = actWinC(1) + 1;
                else
                    actLossC(1) = actLossC(1) + 1;
                end
            end
            lossCount = [0,0];
            winCount = [0,0];

        end
       %         loopActive = true;

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

                
    
            %else
            
                
    
            %end
            
    
            
           
    %toc

    
end
k = 1;
for i = 1:10:numScenarios-10
    [G,H] = find(RewardSet(1:i) < min(matrix(1,:))/2);
    cumulCollisionPercent(k) = 100*size(H,2)/size(RewardSet(1:i),2);
    [G,H] = find(ActionSet(1:i) == 0);
    cumulSwitchPercent(k) = 100*size(H,2)/size(ActionSet(1:i),2);
    [G,H] = find(RewardSet(i:i+10-1) < min(matrix(1,:))/2);
    colPercent(k) = 100*size(H,2)/10;
    k = k + 1;
end

figure
subfigure(1,2,1)
plot(cumulCollisionPercent)
xlabel('Iterations (X10)')
ylabel('Cumulative collision percentage')

subplot(1,2,2)
plot(cumulSwitchPercent)
xlabel('Iterations (X10)')
ylabel('Cumulative switch percentage')

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

function A = getUserInputArray()
    prompt = {'Enter row Collision Yes Switch Yes Collision No Switch Yes (e.g., [1 2]):', 'Enter row Collision Yes Switch No Collision No Switch No (e.g., [3 4]):'};
    dlg_title = 'Input 2×2 Matrix';
    num_lines = [1 50];  % Single-line input
    defaultans = {'[-11 -1]', '[-10 0]'};  % Default input
    answer = inputdlg(prompt, dlg_title, num_lines, defaultans);
    
    if isempty(answer)  % If user cancels
        A = [];
        return;
    end
    
    % Convert string input to numerical array
    row1 = str2num(answer{1}); %#ok<ST2NM>
    row2 = str2num(answer{2}); %#ok<ST2NM>
    
    % Validate input
    if numel(row1) ~= 2 || numel(row2) ~= 2
        error('Invalid input! Please enter two numbers per row.');
    end
    
    A = [row1; row2];  % Create the 2×2 matrix
end


function choice = getUserChoice()
    % Create a dialog box with three options
    answer = questdlg('Choose an option:', 'Selection', 'Epsilon greedy', 'Non Stationary', 'Optimal', '');

    % Assign value based on selection
    switch answer
        case 'Epsilon greedy'
            choice = 1;
        case 'Non Stationary'
            choice = 0;
        case 'Optimal'
            choice = 2;
        otherwise
            choice = 1; % Handles case where user closes the dialog
    end
end

function [choice, meanRT, stdRT] = getHumanChoice()
% Initialize default values for meanRT and stdRT
    meanRT = NaN;
    stdRT = NaN;
    % Create a dialog box with three options
    % Create a dialog box with three options
    answer = questdlg('Choose an option:', 'Selection', 'Sluggish Strong User', 'Fast Weak User', 'Custom Reaction Time', 'Sluggish Strong User');
    
    % Assign value based on selection
    switch answer
        case 'Sluggish Strong User'
            choice = 1;
        case 'Fast Weak User'
            choice = 0;
        case 'Custom Reaction Time'
            % Ask for mean reaction time and standard deviation
            prompt = {'Enter mean reaction time:', 'Enter standard deviation:'};
            dlgtitle = 'Input';
            dims = [1 35];
            definput = {'0', '0'};
            userInput = inputdlg(prompt, dlgtitle, dims, definput);
            
            % Convert user input to numerical values
            if ~isempty(userInput)
                meanRT = str2double(userInput{1});
                stdRT = str2double(userInput{2});
            end
            choice = 2; % Indicating custom reaction time choice
        otherwise
            choice = 1; % Handles case where user closes the dialog
    end
end


%save('DataForScenarios.mat','collideVal','IODataForSurrogate')

