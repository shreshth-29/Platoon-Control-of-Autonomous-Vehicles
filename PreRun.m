Ts = 1e-2;
T = 80;
G_ego = tf(1,[0.5,1,0]);
x0_lead = 70;   % initial position for lead car (m)
v0_lead = 25;   % initial velocity for lead car (m/s)

x0_ego = 10;   % initial position for ego car (m)
v0_ego = 20;   % initial velocity for ego car (m/s)
t_gap = 1.4;
D_default = 10;
v_set = 30;
amin_ego = -3;
amax_ego = 2;
v0_ego_1 = 20;   % initial velocity for ego car (m/s)
v0_ego_2 = 20;   % initial velocity for ego car (m/s)
v0_ego_3 = 20;   % initial velocity for ego car (m/s)
x0_ego_1 = x0_lead - (10 + t_gap*v0_ego_1);   % initial position for ego car (m)
x0_ego_2 = x0_ego_1 - (10 + t_gap*v0_ego_2);   % initial position for ego car (m)
x0_ego_3 = x0_ego_2 - (10 + t_gap*v0_ego_3);   % initial position for ego car (m)

y0_ego_1 = 0;
o0_ego_1 = 0;
y0_lead = 0;
o0_lead = 0;