function plotEdgelets(edgelets,color)
        for i=1:length(edgelets)
            %if abs(edgelets(i).theta)>10, continue;end;                         
            vPts =  [edgelets(i).vPts_un];            
            if isempty(vPts), continue; end;      
            plot(vPts(1,:), vPts(2,:), 'Color',  color, 'LineWidth', 2 );
        end
end