%% Homework 1 Programming Assignment Problem 3
%% User input parameters
T = input('Spectify  T:');
q = input('Specify q:');
s_hat = input('Specify s_hat:');
h = input('Specify h:');
theta = input('Specify theta: ');
%% Uncomment code below for example case (Parameters below are the test case from the problem)
% T = [1 0 0 2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
% q = [0; 2 ; 0];
% s_hat = [0; 0 ; 1];
% h = 2;
% theta = pi;
 %% Solve for T1
V = [s_hat; -cross(s_hat,q)+h*s_hat]; % compute screw
theta_dot = norm(V(1:3),2);
S = V/theta_dot;
w = S(1:3);
v = S(4:6); 

configs = theta*(0:1/4:1); % specify begin, intermediate and end configuration
res = zeros(4,4,length(configs)); % store configuration for each step
figure;
hold on;
for ii = 1:length(configs)
    iTheta = configs(ii);
    w_hat = skewSymm(w); % convert R^3 vector to skew-symmetric matrix
    R = eye(3) + w_hat*sin(iTheta) + w_hat^2*(1-cos(iTheta)); % Rotaion Exponential
    p = (eye(3)*iTheta + (1-cos(iTheta))*w_hat + (iTheta - sin(iTheta))*w_hat^2)*v;
    T1 = [R p;zeros(1,3) 1]*T; % obtain body configuration
    res(:,:,ii) = T1; % store configuration data
    pTheta = T1(1:3,4); % displacement vector in transformation matrix
    
    %%plot coordinates configuration
    quiver3(pTheta(1),pTheta(2),pTheta(3),R(1,1),R(2,1),R(3,1),2,'r','lineWidth',2,'MaxHeadSize',0.5);
    quiver3(pTheta(1),pTheta(2),pTheta(3),R(1,2),R(2,2),R(3,2),2,'g','lineWidth',2,'MaxHeadSize',0.5);
    quiver3(pTheta(1),pTheta(2),pTheta(3),R(1,3),R(2,3),R(3,3),2,'b','lineWidth',2,'MaxHeadSize',0.5);
    text(pTheta(1),pTheta(2),pTheta(3), sprintf('step = %d',ii)); %specify steps
end
lim = [min(min(res(:,4,:)))-2, max(max(res(:,4,:)))+2]; % scale plot
xlim(lim);
ylim(lim);
zlim(lim);
grid on;
view(3);
xlabel('x');
ylabel('y');
zlabel('z');

%% Solver for S1 and theta 1
% Tx Ttheta = T0
RTheta = T1(1:3,1:3); % rotation matrix R in T1
T0 = eye(4); % target configuration at origin
Tx = T0*[RTheta' -RTheta'*pTheta; zeros(1,3) 1]; % solve for space frame transformation Tx
Rx = Tx(1:3,1:3); % rotaion part of transformation matrix
px = Tx(1:3,4); % translation part
if isequal(Rx,eye(3)) % check if it is a pure translation(refer to page 10 in W4-L1)
    w_x = zeros(3,1);
    v_x = px/norm(px,2);
    theta_x = norm(px,2);
else
    [w_x,theta_x] = cvt_R2rotvec(Rx);
    invG = eye(3)/theta_x - 1/2*skewSymm(w_x) + (1/theta_x - 1/2*cot(theta_x/2))*skewSymm(w_x)^2;
    v_x = invG*px;
end
S1 = [w_x; v_x]; % Solution to S1
quiver3(pTheta(1),pTheta(2),pTheta(3),v_x(1),v_x(2),v_x(3),0,'m','lineWidth',2,'MaxHeadSize',2); % plot v
quiver3(0,0,0,w_x(1),w_x(2),w_x(3),3,'k','lineWidth',2,'MaxHeadSize',0.5); % plot w