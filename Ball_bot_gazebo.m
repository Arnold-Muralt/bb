rosinit;
mk=0.326; rk=0.08; Ik=0.00139093;
ma=1.573;Ix=0.1; Iy=0.1; Iz=0.007;
rw=0.03;Iw=0.0000275;l=0.08;g=9.81;
alpha=deg2rad(45);

wheel1_torque=rospublisher('/ballbot/joints/wheel1_effort_controller/command');
wheel2_torque=rospublisher('/ballbot/joints/wheel2_effort_controller/command');
wheel3_torque=rospublisher('/ballbot/joints/wheel3_effort_controller/command');
pose=rossubscriber('/ballbot/sensor/imu');
ball_position=rossubscriber('/gazebo/model_states');
torque1=rosmessage(wheel1_torque);
torque2=rosmessage(wheel2_torque);
torque3=rosmessage(wheel3_torque);
torque1.Data=0;
torque1.Data=0;
torque1.Data=0;
send(wheel1_torque,torque1);
send(wheel2_torque,torque2);
send(wheel3_torque,torque3);
tic;
t=0;
k1=[-0.7167    8.6648   -1.4931    3.0006
   -0.7167    8.6648   -1.4931    3.0006];
k2=[2.0000    3.0000];
multi=[(2/(3*cos(alpha))),0,1/(3*sin(alpha));
       (-1/(3*cos(alpha))),sqrt(3)/(3*cos(alpha)),1/(3*sin(alpha));
       (-1/(3*cos(alpha))),-1*sqrt(3)/(3*cos(alpha)),1/(3*sin(alpha))]
while(t<20)
    t=toc;
    position=receive(ball_position);
    [ground_postion,ballbot_position,ballGround_position]=position.Pose.Position;
    [ground_velocity,ballbot_velocity,ball_velocity]=position.Twist.Linear;
    imu=receive(pose);
    ballbot_orientation=[imu.Orientation.W, imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z];
    ballbot_orientation=rad2deg(quat2eul(ballbot_orientation,'XYZ'));
    ballbot_angularVelocity=[imu.AngularVelocity.X, imu.AngularVelocity.Y,imu.AngularVelocity.Z];
    ball_velocity
    yk=ballbot_position.Y;
    xk=ballbot_position.X;
    yk_dot=ball_velocity.Y;
    xk_dot=ball_velocity.X;
    theta_x=ballbot_orientation(1);
    theta_y=ballbot_orientation(2);
    theta_z=ballbot_orientation(3);
    theta_xdot=ballbot_angularVelocity(1);
    theta_ydot=ballbot_angularVelocity(2);
    theta_zdot=ballbot_angularVelocity(3);
    
    v1=-k1*([yk;theta_x;yk_dot;theta_xdot]);
    v2=-k1*([xk;theta_y;xk_dot;theta_ydot]);
    v3=-k2*([theta_z;theta_zdot]);
    
    wheel_torque=multi*[v1(1);v2(1);v3(1)];
    torque1.Data=wheel_torque(1)/10;
    torque2.Data=wheel_torque(2)/10;
    torque3.Data=wheel_torque(3)/10;
    send(wheel1_torque,torque1);
    send(wheel2_torque,torque2);
    send(wheel3_torque,torque3); 
end
torque1.Data=0;
torque2.Data=0;
torque3.Data=0;
send(wheel1_torque,torque1);
send(wheel2_torque,torque2);
send(wheel3_torque,torque3);
rosshutdown;
    
    



