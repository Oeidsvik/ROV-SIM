function [A_w] = ElementWaveAcc(W_a,N)

%% Element mean acceleration
count=2;
for i=10:5:(N-1)*5
    A_w(count,1) = (W_a(i-1))/2 +(W_a(i-6))/2;                  
    A_w(count,2) = (W_a(i))/2+(W_a(i-5))/2;          
    A_w(count,3) = (W_a(i+1))/2+(W_a(i-4))/2;      
    count=count+1;
end



% end elements
A_w(1,1)       =  (W_a(4))/2     +(W_a(1))/2;        
A_w(1,2)       =  (W_a(5))/2     +(W_a(2))/2;             
A_w(1,3)       =  (W_a(6))/2     +(W_a(3))/2;          
A_w(N,1)       =  (W_a(N*5-1))/2 + (W_a(N*5-6))/2;
A_w(N,2)       =  (W_a(N*5))/2   + (W_a(N*5-5))/2;
A_w(N,3)       =  (W_a(N*5+1))/2 +  (W_a(N*5-4))/2;
end

