clc
clear
close all

l1 = 0.6; l2 = 0.4; l3 = 0.3;
b1 = 0.4;
phi = pi/6; psi = pi/6;

WLrover = WLRobot('Putnik');

Names = {'FR','RR','RL','FL'};

%R
A{1} = [l1 l2 l3*cos(psi) 0]';
D{1} = [-b1 0 -l3*sin(psi) 0]';
Alpha{1} = [-pi/2 0 -pi/2 pi/2]';
Theta{1} = [-phi (pi-phi) 0 0]';

A{2} = [-l1 l2 l3*cos(psi) 0]';
D{2} = [-b1 0 -l3*sin(psi) 0]';
Alpha{2} = [-pi/2 0 pi/2 -pi/2]';
Theta{2} = [-(pi-phi) (pi-phi) 0 0]';
%L
A{3} = [-l1 l2 l3*cos(psi) 0]';
D{3} = [b1 0 -l3*sin(psi) 0]';
Alpha{3} = [-pi/2 0 pi/2 -pi/2]';
Theta{3} = [-(pi-phi) (pi-phi) 0 0]';

A{4} = [l1 l2 l3*cos(psi) 0]';
D{4} = [b1 0 -l3*sin(psi) 0]';
Alpha{4} = [-pi/2 0 -pi/2 pi/2]';
Theta{4} = [-phi (pi-phi) 0 0]';

WLrover.CreateKinematicTree(A,D,Alpha,Theta,Names);

    qFR = [phi pi-phi 0 0];
    qRR = [(pi-phi) -(pi-phi) 0 0];
    qRL = [pi-phi -(pi-phi) 0 0];    
    qFL = [phi pi-phi 0 0];
    q = [qFR; qRR; qRL; qFL;]
    WLrover.SetCurrentConfiguration(q);
    WLrover.ShowFrame(q);
%  show(WLrover.tree,[qFR qRR qRL qFL]);
 
 axes1 = gca;
view(axes1,[43.6191216498061 23.0949018021937]);
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'CameraViewAngle',3.10364992073391,'DataAspectRatio',[1 1 1],...
    'Projection','perspective');

% WLrover.activeLegs = [1 1 1 1];
t = tic
WLrover.ExecuteMotion('Along',1)
a = toc(t)