%% Calling simulink model and security

clc
clear all

close all

%% Conditions for rainy road
Gain = 5000;
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
            
%% Membership for H, M, L braking

load('MemberDecel150.mat')

meanVal = [mean(decelMax(1:13)) mean(decelMax(14:28)) mean(decelMax(29:41))];
stdVal = [std(decelMax(1:10)) std(decelMax(11:30)) std(decelMax(31:41))];
for i = 1:3
    if(i == 1)
        y(i,:) = 1-normcdf(0:150,meanVal(i),stdVal(i));
    elseif(i == 2)
        
        y(i,:) = normpdf(0:150,meanVal(i),stdVal(i));
        y(i,:) = y(i,:)/max(y(i,:));
    else

        y(i,:) = normcdf(0:150,meanVal(i),stdVal(i));
    end
end

plot(y')