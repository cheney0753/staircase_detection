function tracks=trackStep2(tracksPre, observations,  bBoxObservations, NumAtPeak, stepHeight, xyzGP, xyzGPPre)
maxID=max([tracksPre(:).id]);
tracks=initializeTrack();



%% separate the active and inactive tracks pre
isActiveTracksPre=[tracksPre(:).isActive]==1;
tracksActive=tracksPre(isActiveTracksPre);
tracksInactive=tracksPre(~isActiveTracksPre);
%% predict new feature plane of existing tracks
for i_ex=1:length(tracksActive)
   predictedFearture=predict(tracksActive(i_ex).featureFilter);
   predictedBBox=predict(tracksActive(i_ex).bBoxFilter);
end
%% associate observations and tracksPre

[assignments, unassignedTracks, unassignedDetections]=...
    observation2trackAssignment(tracksActive,bBoxObservations);

%% update from tracksPre to tracks

% update for those assigned tracks
numAssignedTracks=size(assignments, 1);

for i_as=1: numAssignedTracks
    idTrack=assignments(i_as,1);
    idObservation=assignments(i_as,2);
    % correct the estimate of plane using the new detection
    correctedBBox=correct(tracksActive(idTrack).bBoxFilter,bBoxObservations(idObservation,:));
    correctedFeature=correct(tracksActive(idTrack).featureFilter,observations(idObservation,:));
    newTrack=struct(...
        'id', tracksActive(idTrack).id, ...   
        'NumAtPeak', NumAtPeak(idObservation),...
        'bBox', correctedBBox, ...
        'bBoxFilter', tracksActive(idTrack).bBoxFilter, ...
        'feature', correctedFeature, ...
        'featureFilter', tracksActive(idTrack).featureFilter,...
        'age',tracksActive(idTrack).age+1, ...
        'totalVisibleCount', tracksActive(idTrack).totalVisibleCount+1, ...
        'consecutiveInvisibleCount', 0, ...
        'isActive', 1);
    tracks(idTrack)=newTrack;
   
 end

% update for those unassigned tracks
for i_unTr=1:length(unassignedTracks)
    idTrackUn=unassignedTracks(i_unTr);
    newTrack=tracksActive(idTrackUn);
    newTrack.bBox=tracksActive(idTrackUn).bBoxFilter.State([1 3 5 7])';
    newTrack.NumAtPeak=tracksActive(idTrackUn).NumAtPeak;
    %if tracksActive(idTrackUn).id==min([tracksActive(:).id]);
    %{
    if idTrackUn==length(tracksActive) && idTrackUn>1 % if the missing track is the last track in trackActive
    
        for h=length(unassignedTracks):idTrackUn
            if tracks(idTrackUn-h).consecutiveInvisibleCount==0
                IDdiff=tracks(idTrackUn-h).id-tracksActive(idTrackUn).id;
                FeatureUnassignedPredicted=...
                    [tracks(idTrackUn-h).feature(1:3) ...
                    tracks(idTrackUn-h).feature(4)+IDdiff*stepHeight];
            break;
            end
            
    end
    
        newTrack.feature=FeatureUnassignedPredicted;
    %else
    %}
        newTrack.feature=tracksActive(idTrackUn).featureFilter.State([1 3 5 7])';
    %end
    newTrack.consecutiveInvisibleCount=...
        newTrack.consecutiveInvisibleCount+1;
    newTrack.age=newTrack.age+1;
    tracks(idTrackUn)=newTrack;
end

% create new tracks for those unassigned observations
for i_unOb=1:length(unassignedDetections)
    idObservUn=unassignedDetections(i_unOb);
    idNext=maxID+i_unOb;
    newFeatureFilter=configureKalmanFilter('ConstantVelocity',...
        observations(idObservUn,:),[200, 25], [100, 12.5], 10);
    newbBoxFilter=configureKalmanFilter('ConstantVelocity',...
        bBoxObservations(idObservUn,:),[200, 50], [100, 25], 100);
    newTrack=struct(...
        'id', idNext, ...
        'NumAtPeak', NumAtPeak(idObservUn),...
        'bBox', bBoxObservations(idObservUn,:), ...
        'bBoxFilter', newbBoxFilter, ...
        'feature', observations(idObservUn,:), ...
        'featureFilter', newFeatureFilter, ...
        'age',1, ...
        'totalVisibleCount', 1, ...
        'consecutiveInvisibleCount', 0, ...
        'isActive', 1);
    tracks=[tracks newTrack];
end


%% remove the tracks that is invisible for n frames to tracksLost
    lostID=removeLostTracks();
    function lostID=removeLostTracks()
        
        if isempty(tracks)
            return;
        end
        
        
        invisibleThreshold=4;
        lostID=find([tracks(:).consecutiveInvisibleCount] >=invisibleThreshold);
        
        %remove the lost tracks
        
        for e=1:length(lostID)
            tracks(lostID(e)).isActive=0;
        end
        
        
    end


%% remove the tracks Lost elements which is not close to the edge of image
    removeFakeLostTracks();
    function removeFakeLostTracks()
       
        if isempty(lostID)
            return;
        end
        
        fakeID=[];
        for fl=1:length(lostID)
            if (480-tracks(fl).bBox(2))>60
                fakeID=[fakeID fl];
            end
        end
            tracks=tracks(setdiff(1:length(tracks), fakeID));
            
    end

%% delete the tracks that is not consequtively visible
deleteTracks();
    function deleteTracks()
        
       if isempty(tracks)
           return;
       end
       
       ageThreshold=8;
       
       ages = [tracks(:).age];
       totalVisibleCounts = [tracks(:).totalVisibleCount];
       visibility = totalVisibleCounts ./ ages;
       
       deleteID=(ages < ageThreshold & visibility < 0.6) ;
       
       %delete 
       tracks=tracks(~deleteID);
        
    end

%% sort the tracks w.r.t the distance to camera
sortTracks();
    function sortTracks()
        
        if isempty(tracks)
            return;
        end
        
        a=reshape([tracks(:).feature],4, length(tracks))';
        [ sortResult, sortList] = sort(a(:,4));
        
        tracks=tracks(sortList);
        
    end

%%



%%
    function [assignments, unassignedTracks, unassignedDetections]=...
        observation2trackAssignment(tracksActive,bBoxObservations)

        assignments=[];
        unassignedTracks=[];
        unassignedDetections=[];

        nTracks= length(tracksActive);

        nObservations= size(bBoxObservations,1);
        
        cost1=zeros(nTracks, nObservations);
        cost2=zeros(nTracks, nObservations);

    for i=1:nTracks
        for j=1:nObservations        
            cost1(i,j)= costCompute(tracksActive(i).bBoxFilter.State([1 3 5 7])', bBoxObservations(j,:));
        end
    end
    
    for i=1:nTracks
        for j=1:nObservations        
            cost2(i,j)= costCompute2(tracksActive(i).featureFilter.State([1 3 5 7]'), observations(j,:));
        end
    end
    
    for i=1:nTracks
        for j=1:nObservations        
            cost3(i,j)= costCompute3(tracksActive(i).NumAtPeak, NumAtPeak(j));
        end
    end
    
    
    cost=0.5*cost1/norm(cost1)+0.3*cost2/norm(cost2)+0.2*cost3/norm(cost3);
    
    costOfNonAssignment=0.10;

    % use the Munkres algorithm to solve the assignment problem
    [assignments, unassignedTracks, unassignedDetections]=assignDetectionsToTracks(cost, costOfNonAssignment);

end
%%

function costij=costCompute(tracki, observationj)
    cTracki=[tracki(1)+tracki(3)/2  tracki(2)+tracki(4)/2];
    cObservation=[observationj(1)+observationj(3)/2 observationj(2)+observationj(4)/2];
    costij=norm(cTracki-cObservation); 
end

function costij=costCompute2(tracki, observationj)
        costij=abs(tracki(4)-observationj(4)); 
end

function costij=costCompute3(tracki, observationj)
        costij=abs(tracki-observationj); 
end


tracks=[tracks tracksInactive];
end