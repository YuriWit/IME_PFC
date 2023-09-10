%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test Case 5
% Stationary four rotor quadcopter with a simgle spining rotor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Global and Radar Setup
setup;

%% Taget Setup

% Four Rotor Quadcopter Target
% tp for targetParams
tp.c = c;
tp.fc = fc;
tp.meanBodyRCS = 1; % mean radar cross section (m^2)
tp.meanBladeRCS = 1; % mean radar cross section (m^2)
tp.pointsPerBlade = 1;

% relative position velocity vectors for the rotors (m)
tp.rotor1RelativePosition = .15*[1;1;0];
tp.rotor2RelativePosition = .15*[-1;1;0];
tp.rotor3RelativePosition = .15*[-1;-1;0];
tp.rotor4RelativePosition = .15*[1;-1;0];

% radius vectors (m^3)
r = .331/2; a1 = 30; a2 = 60; a3 = 90; a4 = 120;
tp.rotor1RadiusVector = r*[cos(a1*pi/180);sin(a1*pi/180);0];
tp.rotor2RadiusVector = r*[cos(a2*pi/180);sin(a2*pi/180);0];
tp.rotor3RadiusVector = r*[cos(a3*pi/180);sin(a3*pi/180);0];
tp.rotor4RadiusVector = r*[cos(a4*pi/180);sin(a4*pi/180);0];

% angular velocity vectors (rad/s)
tp.rotor1AngularVelocityVector = [0;0;3000] *2*pi/60;
tp.rotor2AngularVelocityVector = [0;0;1e-9] *2*pi/60;
tp.rotor3AngularVelocityVector = [0;0;1e-9] *2*pi/60;
tp.rotor4AngularVelocityVector = [0;0;1e-9] *2*pi/60;

tp.position = [-250;0;0]; % position vector (m)
tp.velocity = [0;0;0]; % velocity vector (m/s)

target = QuadcopterTarget(tp);

%% Simulate

simulate;
