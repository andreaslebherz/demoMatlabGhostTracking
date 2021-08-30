classdef helperGIFWriter < matlab.System
   properties
       Figure
       RecordGIF = false;
       DownsampleFactor = 1;
   end
   
   properties
       pFrames = {};
   end
   
   methods
       function obj = helperGIFWriter(varargin)
          setProperties(obj,nargin,varargin{:}); 
       end
       
       function writeAnimation(obj,fName)
           if obj.RecordGIF
               frames = obj.pFrames;
               imSize = size(frames{1}.cdata);
               im = zeros(imSize(1),imSize(2),1,floor(numel(frames)/obj.DownsampleFactor),'uint8');
               map = [];
               count = 1;
               for i = 1:obj.DownsampleFactor:numel(frames)
                   if isempty(map)
                       [im(:,:,1,count),map] = rgb2ind(frames{i}.cdata,256,'nodither');
                   else
                       im(:,:,1,count) = rgb2ind(frames{i}.cdata,map,'nodither');
                   end
                   count = count + 1;
               end
               imwrite(im,map,[fName,'.gif'],'DelayTime',0,'LoopCount',inf);
           end
       end
   end
   
   methods (Access = protected)
       function stepImpl(obj)
           if obj.RecordGIF
               obj.pFrames{end+1} = getframe(obj.Figure);
           end
       end
   end
end