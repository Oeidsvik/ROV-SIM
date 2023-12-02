function [K_e_elem_1,M_e_elem_1] = matrix_update(A,E,l,Delta_length,I,M,M_A,gamma,E_max,E_length_init)

Delta_length=Delta_length(4);
%% Check whether a new element is to be introduced or not...
if  Delta_length>=abs((E_max-1)*E_length_init);
    l2=0;
else
    l2=l;
end
%% Stiffness 
% Axial stiffness
K_e_a_new=[A*E/(l2+Delta_length),0,0,0,0,-A*E/(l2+Delta_length),0,0,0,0;
           0,0,0,0,0,0,0,0,0,0;
           0,0,0,0,0,0,0,0,0,0;
           0,0,0,0,0,0,0,0,0,0;
           0,0,0,0,0,0,0,0,0,0;
           -A*E/(l2+Delta_length),0,0,0,0,A*E/(l2+Delta_length),0,0,0,0;
           0,0,0,0,0,0,0,0,0,0;
           0,0,0,0,0,0,0,0,0,0;
           0,0,0,0,0,0,0,0,0,0;
           0,0,0,0,0,0,0,0,0,0];

% bending stiffness
K_e_e_new=[0,0,0,0,0,0,0,0,0,0;   
    0,12*E*I(2)/(l2+Delta_length)^3,0,0,6*E*I(2)/(l2+Delta_length)^2,0,-12*E*I(2)/(l2+Delta_length)^3,0,0,6*E*I(2)/(l2+Delta_length)^2;
     0,0,12*E*I(3)/(l2+Delta_length)^3,-6*E*I(3)/(l2+Delta_length)^2,0,0,0,-12*E*I(3)/(l2+Delta_length)^3,-6*E*I(2)/(l2+Delta_length)^2,0; 
    0,0,-6*E*I(3)/(l2+Delta_length)^2,4*E*I(3)/(l2+Delta_length),0,0,0,6*E*I(3)/(l2+Delta_length)^2,2*E*I(3)/(l2+Delta_length),0;
    0,6*E*I(2)/(l2+Delta_length)^2,0,0,4*E*I(2)/(l2+Delta_length),0,-6*E*I(2)/(l2+Delta_length)^2,0,0,2*E*I(2)/(l2+Delta_length);
    0,0,0,0,0,0,0,0,0,0;
    0,-12*E*I(2)/(l2+Delta_length)^3,0,0,-6*E*I(2)/(l2+Delta_length)^2,0,12*E*I(2)/(l2+Delta_length)^3,0,0,-6*E*I(2)/(l2+Delta_length)^2;
    0,0,-12*E*I(3)/(l2+Delta_length)^3,6*E*I(3)/(l2+Delta_length)^2,0,0,0,12*E*I(3)/(l2+Delta_length)^3,6*E*I(2)/(l2+Delta_length)^2,0;
    0,0,-6*E*I(3)/(l2+Delta_length)^2,2*E*I(3)/(l2+Delta_length),0,0,0,6*E*I(3)/(l2+Delta_length)^2,4*E*I(3)/(l2+Delta_length),0;
    0,6*E*I(2)/(l2+Delta_length)^2,0,0,2*E*I(2)/(l2+Delta_length),0,-6*E*I(2)/(l2+Delta_length)^2,0,0,4*E*I(2)/(l2+Delta_length)];

K_e_elem_1=K_e_e_new+K_e_a_new;

%% Mass matrix
 M_e_elem_1=zeros(10,10);
    for i=1:5:6
     M_e_elem_1(i,i) =         (M)*(l2+Delta_length)*0.5;
     M_e_elem_1(i+1,i+1) = (M+M_A)*(l2+Delta_length)*0.5;
     M_e_elem_1(i+2,i+2) = (M+M_A)*(l2+Delta_length)*0.5;   
     M_e_elem_1(i+3,i+3) =   0.5*M*(l2+Delta_length)*(l2+Delta_length)^2*gamma/210;
     M_e_elem_1(i+4,i+4)  =  0.5*M*(l2+Delta_length)*(l2+Delta_length)^2*gamma/210; 
    end    
end




