function a = defuzzBrake(yB)

load('MemberDecel200.mat')

meanVal = [mean(decelMax(1:13)) mean(decelMax(14:28)) mean(decelMax(29:41))];
stdVal = [std(decelMax(1:10)) std(decelMax(11:30)) std(decelMax(31:41))];

rangeD = 0:200;

y(1,:) = 1 - normcdf(rangeD,meanVal(1),stdVal(1));

[G,H] = find(y(1,:)>yB(1));

y(1,H) = yB(1);

y(2,:) = normpdf(rangeD,meanVal(2),stdVal(2));
y(2,:) = y(2,:)/max(y(2,:));


[G,H] = find(y(2,:)>yB(2));

y(2,H) = yB(2);

y(3,:) = normcdf(rangeD,meanVal(3),stdVal(3));

[G,H] = find(y(3,:)>yB(3));

y(3,H) = yB(3);


%figure

for i = 1:size(y,2)
    overallY(i) = max(y(:,i));

end

%plot(overallY)

a = defuzz(rangeD,overallY,'bisector');