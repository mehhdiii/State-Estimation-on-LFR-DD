function simulation
% Goal: the purpose of this program was to
% keep the robot follow an elliptic contour.
% An ellipse is generalized enough to include
% circular contour. Result: Mission achived.

close all
T = 0.3;
global s d b r bb q ellipse_a ellipse_b
N = 1e3;
d = 2;
b = 2;
s = b/3;
d1 = d/3;
d2 = d/2;
r = 5*d;
rw = 1/2;
q = 0.85;
ellipse_a = 1;
ellipse_b = 1.2;
bb = (ellipse_b+ellipse_a/2)*r;

% Body base position in global frame of reference
Body = [-b b -d d];
Body_X = [Body(3) Body(3) Body(4) Body(4)];
Body_Y = [Body(1) Body(2) Body(2) Body(1)];

% Sensors base position in global frame of reference
Sen_X = [-d1 -d1 d2];
Sen_Y = [s -s 0];

theta=linspace(0,2*pi,100); % length is fixed
% Robot is placed at (x,y) = (r,0)
x_pos = rand*10;
y_pos = rand*10;
phi = (2*rand-1)*pi;
dd = 20/180*pi; % deg to rad (90 deg/sec)

%KF storage variables: 
xhat_last = [x_pos; y_pos; phi];

% dev_w = 1e-2;
% dev_v = 1e-2;
trajectory =zeros(2,N);
trajectory_KF = zeros(2, N); 

RC_Sens = [cos(phi) -sin(phi); ...
    sin(phi) cos(phi)]*[Sen_X; Sen_Y]+[x_pos;y_pos];

%stochastic setup:
kR = 1e-2; kL = 1e-2;
P_last = eye(3);

%stochastic setup for observation: 
R_k = diag([1e-3 1e-3]);

for ii = 1:N
    
    %calculate sensor readings:
    [fL,fR,fC]=check_sensors2(RC_Sens(1,:),RC_Sens(2,:));
    
    %control commands for wheels:
    [delR, delL] = LFR_controller(fL,fR,fC, dd);
    
    %calculate input's Covariance matrix: 
    Q_input = diag([kR*abs(delR) kL*abs(delL)]);
    
    %calculating velocities:
    vee = rw*(delR+delL)/2 +randn*sqrt(Q_input(1,1)); % noise added
    omega = rw*(delR-delL)/2/b +randn*sqrt(Q_input(2,2)); % noise added
    
    %Plant:
    delta_phi = T*omega;
    delta_x = 2*vee/omega*sin(delta_phi/2)*cos(phi+delta_phi/2);
    delta_y = 2*vee/omega*sin(delta_phi/2)*sin(phi+delta_phi/2);
    
    x_pos = x_pos + delta_x;
    y_pos = y_pos + delta_y;
    phi = wrapToPi(phi + delta_phi);

    %Propagating input error covariance:
    sigma_ = diag([kR*delR kL*delL]);
    B = [(1/2)*cos(phi+T*omega/2)-(T*vee/(2*b))*sin(phi+T*omega/2), (1/2)*cos(phi+T*omega/2)+(T*vee/(2*b)*sin(phi+T*omega/2));
        (1/2)*sin(phi+T*omega/2)+(T*vee/(2*b))*cos(phi+T*omega/2), (1/2)*sin(phi+T*omega/2)-(T*vee/(2*b))*cos(phi+T*omega/2);
        1/b, -1/b];
    Q_k = B*sigma_*B';
    
    %observation: Assume radar observation 
    y_k = [sqrt(x_pos^2+y_pos^2); atan(y_pos/x_pos)] + sqrt(R_k)*randn(2, 1);
    
    %KF:
    [xhat_optimal,P_optimal] = KalmanFilter(y_k, Q_k, R_k, xhat_last, P_last, vee, omega, T);
    xhat_last = xhat_optimal; 
    P_last = P_optimal;
    
    
    %Transformation matrices: Unfiltered
    RC_Sens = [cos(phi) -sin(phi); ...
        sin(phi) cos(phi)]*[Sen_X; Sen_Y]+[x_pos;y_pos]; 
    
    RC_Body = [cos(phi) -sin(phi); ...
        sin(phi) cos(phi)]*[Body_X;Body_Y]+[x_pos;y_pos];
    
    trajectory(:,ii) =[RC_Sens(1,3); RC_Sens(2,3)];
    
    %Transformation matrices: Filtered
    RC_Sens_KF = [cos(xhat_optimal(3)) -sin(xhat_optimal(3)); ...
        sin(xhat_optimal(3)) cos(xhat_optimal(3))]*[Sen_X; Sen_Y]+[xhat_optimal(1);xhat_optimal(2)]; 
    
    RC_Body_KF = [cos(xhat_optimal(3)) -sin(xhat_optimal(3)); ...
        sin(xhat_optimal(3)) cos(xhat_optimal(3))]*[Body_X;Body_Y]+[xhat_optimal(1);xhat_optimal(2)];

    trajectory_KF(:, ii) = [RC_Sens_KF(1,3); RC_Sens_KF(2,3)];
    
    clf;plot(ellipse_a*(r-s*q)*cos(theta),...
        ellipse_b*(r-s*q)*sin(theta),'k-');hold on
    plot(ellipse_a*(r+s*q)*cos(theta),...
        ellipse_b*(r+s*q)*sin(theta),'k-');hold on
    
    patch(RC_Body(1,:),RC_Body(2,:),'g'); alpha(0.2);
    plot(RC_Sens(1,:),RC_Sens(2,:),...
        'o','markersize',2,'markerfacecolor','k')
    
    patch(RC_Body_KF(1,:),RC_Body_KF(2,:),'g'); alpha(0.2);
    plot(RC_Sens_KF(1,:),RC_Sens_KF(2,:),...
        'o','markersize',2,'markerfacecolor','k')
    axis([-1 1 -1 1]*bb);hold on
    plot(trajectory(1,1:ii),trajectory(2,1:ii),'b-','linewidth',2)
    plot(trajectory_KF(1,1:ii),trajectory_KF(2,1:ii),'r-','linewidth',2)
    axis square; drawnow;
end

