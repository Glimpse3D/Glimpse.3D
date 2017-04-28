mat = csvread('imu_new.csv');
angle = mat(:,11);
time = mat(:,3);
count = 0;
sum = 0;
newtime =[];
array=[];
for i = 2:1000
    if abs(sum)>2
        sum = 0;
        count = count + 1;
        newtime(count)=time(i);
        array(count)=(i-1);
    else
        sum = sum + (angle(i)-angle(i-1));
    end
end
newtime=newtime.';
array=array.';