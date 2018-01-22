function FACADE_plotSegmentation(im,  vsEdges,  vClass, vP_v, vbOutliers, algoNum, noIm)
    
global mClr intrinsics
    only3 = false;
    
    if nargin <= 5
        algoNum = 0; %no save
    end 
    if nargin <= 6
        noIm = false; %no save
    end
    
% $$$     if false %ARGS.only3                
% $$$         vSupport = hist(vClass, 1:max(vClass));
% $$$         [dum, vIdx] = sort(vSupport, 'descend');
% $$$         vClassCpy = vClass;
% $$$         for c=1:length(vIdx)
% $$$             vClassCpy(vClass==vIdx(c)) =c;
% $$$         end
% $$$         vClass = vClassCpy;
% $$$         %vClass(vClass>3) = 0; %eliminate outliers
% $$$     end
    %if ARGS.only3 
    %    vClass(vClass>3) = 0;
    %end
    
    %FCprintf('%d outliers\n', sum(vbOutliers))
    
    nbClass = max(vClass);
    %mClr = FACADE_colorMap(nbClass);
    
    %ploting this
   
    if ~noIm
        imshow(im);
        %imshow(zeros(480,640));
    end
    %edges with color/association
    hold on
    for c=1:nbClass        
        vsEdges_s = [vsEdges(vClass==c)];
        for i=1:length(vsEdges_s)
            if noIm
                vPts =  [vsEdges_s(i).vPts];            
            else
                vPts =  [vsEdges_s(i).vPts_un];            
            end
            if isempty(vPts), continue; end;      
            
            if( c <= 3 ) % 4
            plot(vPts(1,:), vPts(2,:), 'Color',  mClr(c,:), 'LineWidth', 2 );
            end
           
                       
          if( false  && c < 4 && mod(i,2)==0)
            p1 =  mToUh(intrinsics * vP_v(c).VP);
            p2 = vsEdges_s(i).vCt_un;
            
            vLn = [(p2(2)-p1(2))/(p2(1)-p1(1)),-1,p1(2) - (p2(2)-p1(2))/(p2(1)-p1(1))*p1(1)];
            u_line = 1:640;
            v_line = (-vLn(3)-u_line*vLn(1))/vLn(2);
            plot(u_line,v_line,'Color',mClr(c,:));
         
            % vsEdges_s(i).vLn_un = vLn;
          end
            
        end

        if(c==1)
            vp =  mToUh(intrinsics * vP_v(c).VP);
            plot(vp(1),vp(2),'Marker','.','MarkerSize',20,'Color',mClr(c,:));
        end
    end

    hold off
    
    if true
        STEP=1;
        COL = 0;
        MARKER = '.';
    else
        STEP=4;
        COL = 0.3;
        MARKER = 'x';
    end
    
    if only3
        %flag last class as outliers
        hold on;
        vsEdges_s = [vsEdges(vClass>3)];
        if noIm
            vPts =  [vsEdges_s.vPts];
        else
            vPts =  [vsEdges_s.vPts_un];
        end
        
        if ~isempty(vPts)            
            plot(vPts(1,1:STEP:end), vPts(2,1:STEP:end),MARKER, 'Color', [COL,COL,COL] );
        end
        hold off
    end
    
    hold on
    %outliers
    if  ~isempty(vbOutliers) & sum(vbOutliers)>=1
        vsEdges_s = [vsEdges(vbOutliers)];
        if noIm
            vPts =  [vsEdges_s.vPts];      
        else
            vPts =  [vsEdges_s.vPts_un];      
        end
        plot(vPts(1,1:STEP:end), vPts(2,1:STEP:end),'.', 'Color', [0,0,0] );
    end    
    hold off
    
    
    if algoNum >0 %save plot
        filename= sprintf( 'fig/__res_class_%02d.jpg', algoNum);
        %saveas(pFigEdges,filename, 'jpg'); 
        print('-noui' ,'-r0', '-djpeg',  filename);
        %print('-r0', '-djpeg',  filename);
        FCprintf('Wrote %s\n', filename);
    end
    
 
    %done with this plot
    
    drawnow
    return
    