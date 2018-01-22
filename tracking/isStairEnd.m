function isEnd=isStairEnd(tracks, ageThreshold, visibleThreshold)

activeID=([tracks(:).isActive]==1);
activeTracks=tracks(activeID);

activeLength=length(activeTracks);

stability=0;
visibility=0;

ages = [activeTracks(:).age];
totalVisibleCounts = [activeTracks(:).totalVisibleCount];

for h=1:activeLength
    visibility = totalVisibleCounts(h) ./ ages(h); 
    if ages(h)>ageThreshold&&visibility>visibleThreshold
        stability=stability+1;
    end
    
end

if stability==activeLength
    isEnd=1;
else 
    isEnd=0;
end
    
end 