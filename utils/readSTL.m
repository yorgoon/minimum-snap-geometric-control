function [ coordinates] = readSTL( filename)
% reads ASCII STL file and gives coordinates of vertices.
%filename-name of file(test.stl)
fid=fopen(filename);
C=textscan(fid,'%s');
%reads file and generates cell.
k = cellfun(@length,C);
%gives number length of the array.
m = 11;
i = 1;
 while(m < (k-3))
      
   j = 1;
     while (j < 4)    
     l = 1;
             while(l<4)
              coordinates(i,1) = str2double(C{1,1}{m,1}); % took 3 days, 10 odd questions on mathworks.com to write this line :).  
              % data in cells were in 'char' format .convert it to double as
              % matrix cant store 'char's.
              
                l = l+1;  % makes sure loop runs thrice.
                m = m+1;  % access corresponding row from cell'C'.
                i = i+1;  % row number in output matrix.
             
             end 
            
           
      m = m+1;
      j = j+1;
     end
   
   m = m+9;
 
 end
 
 % plotting file using patch command.
 
  S = size(coordinates,1);
  T = S(1,1);
  
  n = 1;
  
 while(n<T)
     o = 1;
     while(o<4)
       D = rand(3,1);
       X(o,1)= coordinates(n,1);
       n = n+1;
       Y(o,1)= coordinates(n,1);
       n = n+1;
       Z(o,1)= coordinates(n,1);
       n = n+1;
       o = o+1;
     end  
     patch(X,Y,Z,D);
     hold on;
     
 end 

 
 
end
