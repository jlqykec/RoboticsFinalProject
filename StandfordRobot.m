classdef StandfordRobot
    properties
        %Links parameters
        alpha1;
        d2;
        alpha2;
        alpha4;
        alpha5;
        %Joints values
        q;           
        %Joints limits
        q_lim;
        
    end
    
    methods (Access = public)  
        
        %Constructor
        function obj=StandfordRobot()
            %Links parameters
            obj.alpha1=-pi/2;
            obj.d2=6.375;
            obj.alpha2=pi/2;
            obj.alpha4=-pi/2;
            obj.alpha5=pi/2;
            %Initial joint values
            obj.q(1)=0;
            obj.q(2)=0;
            obj.q(3)=0;
            obj.q(4)=0;
            obj.q(5)=0;
            obj.q(6)=0;            
            
            %Joints limits
            obj.q_lim=zeros(1,2,6);
            obj.q_lim(1,1,1)=-160*pi/180;
            obj.q_lim(1,2,1)=160*pi/180;
            obj.q_lim(1,1,2)=-125*pi/180;
            obj.q_lim(1,2,2)=125*pi/180;
            obj.q_lim(1,1,3)=-30;
            obj.q_lim(1,2,3)=30;
            obj.q_lim(1,1,4)=-140*pi/180;
            obj.q_lim(1,2,4)=140*pi/180;
            obj.q_lim(1,1,5)=-100*pi/180;
            obj.q_lim(1,2,5)=100*pi/180;
            obj.q_lim(1,1,6)=-260*pi/180;
            obj.q_lim(1,2,6)=260*pi/180;
        end
        
        %Forward Kinematics
        function T6=fwKin(this,q)
            %Check the six joints valus are given
            if size(q,1)~=6
                error('The forward kinematics require 6DOF joints values')
            end
            %Check the input joints values are inside the limits
            if ~(this.checkLimit(q(1),1))
                error('Input value of joint 1 is outside of the limits')
            end
            if ~(this.checkLimit(q(2),2))
                error('Input value of joint 2 is outside of the limits')
            end
            if ~(this.checkLimit(q(3),3))
                error('Input value of joint 3 is outside of the limits')
            end
            if ~(this.checkLimit(q(4),4))
                error('Input value of joint 4 is outside of the limits')
            end
            if ~(this.checkLimit(q(5),5))
                error('Input value of joint 5 is outside of the limits')
            end
            if ~(this.checkLimit(q(6),6))
                error('Input value of joint 6 is outside of the limits')
            end
                        
            %Calculate the trigonometric functions
            C1=cos(q(1));   S1=sin(q(1));
            C2=cos(q(2));   S2=sin(q(2));
            C4=cos(q(4));   S4=sin(q(4));
            C5=cos(q(5));   S5=sin(q(5));
            C6=cos(q(6));   S6=sin(q(6));            
            
            %Initialize transformation matrix
            T6=zeros(4,4);
            
            %orientation direction
            temp1=-C2*(C4*C5*S6+S4*C6)+S2*S5*S6;
            temp2=-S4*C5*S6+C4*C6;
            %ox
            T6(1,2)=C1*temp1-S1*temp2;
            %oy            
            T6(2,2)=S1*temp1+C1*temp2;
            %oz
            T6(3,2)=S2*(C4*C5*S6+S4*C6)+C2*S5*S6;           
            
            %approaching direction
            temp1=C2*C4*S5+S2*C5;
            temp2=S4*S5;
            %ax
            T6(1,3)=C1*temp1-S1*temp2;
            %ay            
            T6(2,3)=S1*temp1+C1*temp2;
            %az
            T6(3,3)=-S2*C4*S5+C2*C5;
            
            %normal direction
            T6(1:3,1)=cross(T6(1:3,2),T6(1:3,3));
            
            %Position
            %px
            T6(1,4)=C1*S2*q(3)-S1*this.d2;
            %py
            T6(2,4)=S1*S2*q(3)+C1*this.d2;
            %pz
            T6(3,4)=C2*q(3);
            T6(4,4)=1;            
        end
        
        %Inverse Kinematics
        function q=invKin(this,T)
            %% Get the vectors from the transformation matrix
            o=T(1:3,2);
            a=T(1:3,3);
            P=T(1:3,4);           
            
            %Initialize the joint solutions, there are four possible solutions
            %The seventh element defines if the solution is valid or not
            sol=zeros(4,7); 
            sol(1:4,7)=1; %All the solutions start with a valid flag
                        
            %% Theta 1 solution
            
            %Initialize theta 1 solutions and their trigonometric functions
            th1=zeros(1,2);
            C1=zeros(1,2);
            S1=zeros(1,2);    
            
            %Calculate theta 1
            r=sqrt(P(1)^2+P(2)^2);
            
            if(r<this.d2)
               error('The result r for calculation of theta 1 is smaller than d2') 
            end
            
            den=sqrt(r^2-this.d2^2);
            phi=atan2(P(2),P(1));            
            temp1=atan2(this.d2,den);
            temp2=atan2(this.d2,-den);            
            
            %Two solutions, left hand shoulder and right hand shoulder
            th1(1)=phi-temp1;
            th1(2)=phi-temp2;    

            %If both solutions are the same eliminate the second
            if(th1(1)==th1(2))
               th1(2)=-99; 
            end
                        
            for i=1:2
                %Copy the solutions checking the joints limits
                if this.checkLimit(th1(i),1)
                    sol(2*i-1:2*i,1)=th1(i);
                    %Calculate the trigonometric functions for theta1
                    C1(i)=cos(th1(i));   S1(i)=sin(th1(i));
                else
                    sol(2*i-1:2*i,7)=0;
                end                
            end
            
            %% Theta 2 and d3 solution
            
            %Initialize theta 2 solutions and their trigonometric functions
            th2=zeros(1,2);
            C2=zeros(1,2);
            S2=zeros(1,2);
            
            %%Initialize d3 solutions
            d3=zeros(1,2);            
            
            for i=1:2 %One solution for each solution of theta 1
                if sol(2*i-1,7)~=0 %Check if the solution is valid first                    
                    %Calculate theta 2
                    th2(i)=atan2(C1(i)*P(1)+S1(i)*P(2),P(3));
               
                    %Copy the solutions checking the joints limits
                    if this.checkLimit(th2(i),2)
                        sol(2*i-1:2*i,2)=th2(i);
                        %Calculate the trigonometric functions for theta 2
                        C2(i)=cos(th2(i));   S2(i)=sin(th2(i));
                    else
                        sol(2*i-1:2*i,7)=0;
                    end   
                    
                    %Calculate d3
                    d3(i)=S2(i)*(C1(i)*P(1)+S1(i)*P(2))+C2(i)*P(3);
               
                    %Copy the solutions checking the joints limits
                    if this.checkLimit(d3(i),3)
                        sol(2*i-1:2*i,3)=d3(i);
                    else
                        sol(2*i-1:2*i,7)=0;
                    end         
                end                
            end
            
            %% Theta 4 solution
            
            %Initialize theta 4 solutions and their trigonometric functions
            th4=zeros(1,4);
            C4=zeros(1,4);
            S4=zeros(1,4);
            
            for i=1:2 %Two solution for each solution of theta 1
                if sol(2*i-1,7)~=0 %Check if the solution is valid first                    
                    %Calculate theta 4
                    num=-S1(i)*a(1)+C1(i)*a(2);
                    den=C2(i)*(C1(i)*a(1)+S1(i)*a(2))-S2(i)*a(3);
                    
                    %Calculate the indexes
                    sol1Idx=2*i-1;
                    sol2Idx=2*i;
                    
                    th4(sol1Idx)=atan2(num,den);
                    th4(sol2Idx)=th4(sol1Idx)+pi;
                    
                    %The second solution of th4 might be greater than 180
                    %For this case change the value to the right range
                    
                    if(th4(sol2Idx)>pi)
                       th4(sol2Idx)=th4(sol2Idx)-2*pi;
                    end                     
                    
                    %Copy the solutions checking the joints limits
                    if this.checkLimit(th4(sol1Idx),4)
                        sol(sol1Idx,4)=th4(sol1Idx);
                        %Calculate the trigonometric functions for theta 4
                        C4(sol1Idx)=cos(th4(sol1Idx));   S4(sol1Idx)=sin(th4(sol1Idx));
                    else
                        sol(sol1Idx,7)=0;
                    end
                    if this.checkLimit(th4(sol2Idx),4)
                        sol(sol2Idx,4)=th4(sol2Idx);
                        %Calculate the trigonometric functions for theta 4
                        C4(sol2Idx)=cos(th4(sol2Idx));   S4(sol2Idx)=sin(th4(sol2Idx));
                    else
                        sol(sol2Idx,7)=0;
                    end
                    
                end                
            end
            
            
            %% Theta 5 solution
            
            %Initialize theta 5 solutions and their trigonometric functions
            th5=zeros(1,4);
            C5=zeros(1,4);
            S5=zeros(1,4);
            
            for i=1:4 
                if sol(i,7)~=0 %Check if the solution is valid first
                    
                    %Indexes for th1 and th2
                    if(i<3)
                        idx1=1;
                    else
                        idx1=2;
                    end
                    
                    %Calculate theta 5                    
                    num=C4(i)*(C2(idx1)*(C1(idx1)*a(1)+S1(idx1)*a(2))-S2(idx1)*a(3))+S4(i)*(-S1(idx1)*a(1)+C1(idx1)*a(2));
                    den=S2(idx1)*(C1(idx1)*a(1)+S1(idx1)*a(2))+C2(idx1)*a(3);
                    
                    th5(i)=atan2(num,den);
                    
                    %If theta 5 is zero change theta 4 to zero and
                    %recalculate theta 5                    
                    if(abs(th5(i))<0.001) %If theta 5 is really close to zero                        
                        sol(i,4)=0;
                        C4(i)=1;    S4(i)=0;                       
                       
                        %Calculate theta 5
                        num=C4(i)*(C2(idx1)*(C1(idx1)*a(1)+S1(idx1)*a(2))-S2(idx1)*a(3))+S4(i)*(-S1(idx1)*a(1)+C1(idx1)*a(2));
                        den=S2(idx1)*(C1(idx1)*a(1)+S1(idx1)*a(2))+C2(idx1)*a(3);
                        
                        th5(i)=atan2(num,den);
                    end
                    
                    %Copy the solutions checking the joints limits
                    if this.checkLimit(th5(i),5)
                        sol(i,5)=th5(i);
                        %Calculate the trigonometric functions for theta 4
                        C5(i)=cos(th5(i));   S5(i)=sin(th5(i));
                    else
                        sol(i,5)=th5(i);
                        sol(i,7)=0;
                    end                    
                end
            end
            
            %% Conditions for sign of theta 5
            %Check the solution of theta 4 is consisten with the
            %sign of theta 5
            if th5(1) < 0.00001
                sol(1,7)=0;
            end
            
            if th5(2) > 0.00001
                sol(2,7)=0;
            end
            
            if th5(3) < 0
                sol(3,7)=0;
            end
            
            if th5(4) > 0
                sol(4,7)=0;
            end            

            %% Theta 6 solution
            for i=1:4 
                if sol(i,7)~=0 %Check if the solution is valid first
                    
                    %Indexes for th1 and th2
                    if(i<3)
                        idx1=1;
                    else
                        idx1=2;
                    end
                    
                    %Calculate theta 6                    
                    temp1=C2(idx1)*(C1(idx1)*o(1)+S1(idx1)*o(2))-S2(idx1)*o(3);
                    temp2=-S1(idx1)*o(1)+C1(idx1)*o(2);
                                       
                    sin6=-C5(i)*(C4(i)*temp1+S4(i)*temp2)+S5(i)*(S2(idx1)*(C1(idx1)*o(1)+S1(idx1)*o(2))+C2(idx1)*o(3));
                    cos6=-S4(i)*temp1+C4(i)*temp2;
                    
                    th6=atan2(sin6,cos6);
                    
                    %Copy the solutions checking the joints limits
                    if this.checkLimit(th6,6)
                        sol(i,6)=th6;
                    else
                        sol(i,7)=0;
                    end
                end                
            end
            
            %% Store the valid solutions
            n=1;
            isThereASol=0;
            q(1,:)=[0 0 0 0 0 0];
            for i=1:4
                if sol(i,7)==1
                    q(n,:)=sol(i,1:6);
                    n=n+1;
                    isThereASol=1;
                end
            end

            sol=sol*180/pi;
            sol(:,3)=sol(:,3)*pi/180
            
            if ~isThereASol
                error('There is not a valid solution');
            end
                                  
        end   
                
    end
    
    methods (Access = private)
        %Function to check the angle is inside the joint range
        function flag=checkLimit(this,angle,jointN)
            if angle>=this.q_lim(1,1,jointN)&&angle<=this.q_lim(1,2,jointN)
                flag=1;
            else
                flag=0;
            end
        end
    end
    
end














