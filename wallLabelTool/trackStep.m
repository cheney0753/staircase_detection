function [tracks, tracksLost]=trackStep(tracksPre, observations, maxID, bBoxObservations, xyzGP, xyzGPPre)

tracks=initializeTracks();
tracksLost=initializeTracks();
%% predict new feature plane of existing tracks
for i_ex=1:length(tracksPre)
   predictedFearture=predict(tracksPre(i_ex).featureFilter);
%   predictedbBoxCentroid=predict(tracksPre(i_ex),bBoxFilter);
   %tracksPre(i_ex).feature=predictedFearture;
   %t 
end


%% associate observations and tracksPre
[assignments, unassignedTracks, unassignedDetections]=...
    observation2trackAssignment(tracksPre,observations);

%% update from tracksPre to tracks

% update for those assigned tracks
numAssignedTracks=size(assignments, 1);

for i_as=1: numAssignedTracks
    idTrack=assignments(i_as,1);
    idObservation=assignments(i_as,2);
    % correct the estimate of plane using the new detection
    correctedFeature=correct(tracksPre(idTrack).featureFilter,observations(idObservation,:));
    newTrack=struct(...
        'id', tracksPre(idTrack).id, ...   
        'bBox', bBoxObservations(idObservation,:), ...
        'feature', correctedFeature, ...
        'featureFilter', tracksPre(idTrack).featureFilter,...
        'age',tracksPre(idTrack).age+1, ...
        'totalVisibleCount', tracksPre(idTrack).totalVisibleCount+1, ...
        'consecutiveInvisibleCount', 0);
    tracks(idTrack)=newTrack;
   
end

% update for those unassigned tracks
for i_unTr=1:length(unassignedTracks)
    idTrackUn=unassignedTracks(i_unTr);
    newTrack=struct(...
        'id', tracksPre(idTrackUn).id, ...
        'bBox', tracksPre(idTrackUn).bBox, ...
        'feature', tracksPre(idTrackUn).feature, ...
        'featureFilter', tracksPre(idTrackUn).featureFilter, ...
        'age',tracksPre(idTrackUn).age+1, ...
        'totalVisibleCount', tracksPre(idTrackUn).totalVisibleCount, ...
        'consecutiveInvisibleCount', ...
            tracksPre(idTrackUn).consecutiveInvisibleCount+1);
    tracks(idTrackUn)=newTrack;
end

% create new tracks for those unassigned observations
for i_unOb=1:length(unassignedDetections)
    idObservUn=unassignedDetections(i_unOb);
    idNext=maxID+1;
    newFeatureFilter=configureKalmanFilter('ConstantVelocity',...
        observations(idObservUn,:),[200, 25], [100, 12.5], 10);
    newTrack=struct(...
        'id', idNext, ...
        'bBox', bBoxObservations(idObservUn,:), ...
        'feature', observations(idObservUn,:), ...
        'featureFilter', newFeatureFilter, ...
        'age',1, ...
        'totalVisibleCount', 1, ...
        'consecutiveInvisibleCount', 0);
    tracks=[tracks newTrack];
end


%% remove the tracks that is invisible for n frames to tracksLost
    removeLostTracks();
    function removeLostTracks()
        
        if isempty(tracks)
            return;
        end
        
        
        invisibleThreshold=4;
        lostID=[tracks(:).consecutiveInvisibleCount] >=invisibleThreshold;
        
        %remove the lost tracks
        
        tracksLost=tracks(lostID);
        tracks=tracks(~lostID);
        
        
    end


%% remove the tracksLost elements which is not close to the edge of image
    removeFakeLostTracks();
    function removeFakeLostTracks()
       
        if isempty(tracksLost)
            return;
        end
        
        fakeID=[];
        for fl=length(tracksLost)
            if (480-tracksLost.bBox(2))>60
                fakeID=[fakeID fl];
            end
        end
            tracksLost=tracksLost(setdiff(1:length(tracksLost), fakeID));
            
    end

%% delete the tracks that is not consequtively visible
deleteTracks();
    function deleteTracks()
        
       if isempty(tracks)
           return;
       end
       
       ageThreshold=5;
       
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

    function tracks=initializeTracks()

    tracks=struct(...
        'id', {}, ...
        'bBox', {}, ...
        'feature', {}, ...
        'featureFilter', {}, ...
        'age', {}, ...
        'totalVisibleCount', {}, ...
        'consecutiveInvisibleCount', {});
    end

%%
    function [assignments, unassignedTracks, unassignedDetections]=...
        observation2trackAssignment(tracksPre,observations)

        assignments=[];
        unassignedTracks=[];
        unassignedDetections=[];

        nTracks= length(tracksPre);

        nObservations= size(observations,1);

        cost=zeros(nTracks, nObservations);

for i=1:nTracks
    for j=1:nObservations
        
        cost(i,j)= costCompute(tracksPre(i).feature, observations(j,:),tracksPre(i).consecutiveInvisibleCount);
    end
end

costOfNonAssignment=0.15;

% use the Munkres algorithm to solve the assignment problem
[assignments, unassignedTracks, unassignedDetections]=assignDetectionsToTracks(cost, costOfNonAssignment);
%{
k_assignment=1;
k_unassignedTracks=1;


for i=1:nTracks
    if  assignmentMatrix(i)~=0
        assignments(k_assignment,:)=[i assignmentMatrix(i)];
        k_assignment=k_assignment+1;
    else
        unassignedTracks(k_unassignedTracks)= i;
        k_unassignedTracks=k_unassignedTracks+1;
	end
end

if nObservations>nTracks
    unassignedDetections=setdiff(1: nObservations, assignmentMatrix);
end
%}
end
%%

function costij=costCompute(tracki, observationj, consecutiveInvisibleCount)
costij=abs(tracki(4)-xyzGPPre(4)-(observationj(4)-xyzGP(4)));%*sqrt(consecutiveInvisibleCount);

end

end