%%asumme in m units
l1 = 0.23241;
l2 = 0.1524;
l3 = 0.0508;
%%DH parameters
H0_1 = Revolute('d', l1 , 'a', 0 , 'alpha', -pi/2);
H1_2 = Revolute('d',0, 'a', -l2 , 'alpha', pi,'offset',pi/2);
H2_3 = Revolute('d', 0, 'a',l2 , 'alpha', 0,'offset',pi/2);
H3_4 = Revolute('d', 0 , 'a', l3, 'alpha', 0);
%%Create serial link manipulator
Agri_bot = SerialLink([H0_1 H1_2 H2_3 H3_4], 'name', 'AgriBot');
%%define joint angles
q_joints = [0,0,0,0];
%%zlim([0, 1500]);
Agri_bot.plot(q_joints);     % Plot the robot in the specified configuration
Agri_bot.teach()
T_end = Agri_bot.fkine(q_joints);  % Compute the forward kinematics to get end-effector pose