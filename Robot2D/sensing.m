function rt = sensing(sense, obs) 
len = length(sense.x);
rt = 0;
for i = 1:1:len
    if(sqrt((sense.x(i) - obs(1))^2 + (sense.y(i) - obs(2))^2) <= 6)
        rt = 1;
        break;
    end
end
end