function [J_space] = J_space(tab_space,config)
%J_SPACE computes the Jacobian matrix for open chain robot defined by end
%screw axes table tab_space; config defines the
%set of joint angles. The output is the Jacobian matrix with respect to
%space frame
n = size(tab_space,1);     %depends on no. of links the robot has
J_space = zeros(6, n);
exp = eye(4);
for i=1:n
    S_i = tab_space(i,:)';
end
J_space(:,1) = S_i(1,:);
      for k=2:n
         exp2 = eye(4);
         for j=1:(k-1)
            w = tab_space(j,1:3);
            v = tab_space(j,4:6); 
            if w == zeros(1,3)                  %if the joint is prismatic
               R = eye(3);
               trans = -config(j)*v';
            else
                R = cvt_rotvec2R(w,-config(j)); %if it is a revolute joint
                w_m = skewSymm(w);
                trans = (eye(3)*-config(j)+(1-cos(-config(j)))*w_m+(-config(j)...
                    -sin(-config(j)))*w_m^4)*v';
            end
            exp =   [R, trans;
                zeros(1,3), 1]
            exp2 = exp2*exp;
         end   
         T = Ad_T(exp2);
         J_space(:,k) = T*S_i(:,k);     %%here is the fault
        end  
    end