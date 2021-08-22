clear all;
close all;
clc;


filepath = '~/Documents/MATLAB/Scripts Robotica/lezione20 Robotica/fanuc_circular.traj';
 
%definiamo pochi punti e poi imponiamo all'interpolatore di generare una
%traiettoria continua che ce li collega. Dato che ROS ha bisogno di una
%sequenza di punti allora ricampioniamo questa curva.
%Quindi definiamo pochi punti, interpoliamo in maniera continua e poi
%andiamo ricampionarla in modo molto più fitto.

no_of_cp = 10; %punti inziali (noi dovremmo mettere il numero 
%minimo di punti che visivamente ci fornisce una circonferenza) 
no_of_samples = 70;%punti a valle della interpolazione. 

rho = 0.5;%raggio costante
%definisco un teta per ogni control_point,
%quindi ho no_of_cp punti con diverse angolazioni di teta e raggio costante
theta = linspace(0, 2*pi, no_of_cp);

ctrl_points = NaN * ones(3, no_of_cp);
ctrl_points(1,:) = 0.5 * ones(1, no_of_cp); % punti rispetto x
%rispetto a x semplicemente dico a che distanza da 
% 0 ho questa circonferenza.

ctrl_points(2,:) = rho * cos(theta);  %punti rispetto y
ctrl_points(3,:) = rho * sin(theta); %punti rispetto a z


%parto da una configurazione polare, 

[x_lambda, lambda] = generate_path(ctrl_points, no_of_samples, true);
%La parte di orientamento non la interpoliamo perché è costante
%nell'esercizio, abbiamo che i sistemi di riferimento in termini di
%orientamento sono concordi (base ed end_effector) ed è per questo che
%l'end effector sta con la flangia verso l'alto. 

%Per portarlo di faccia l'end effector dobbiamo fare una rotazione di pi/2
%su uno solo degli angoli di orientamento.

x_lambda(4:6,:) = zeros(3, no_of_samples);

export_ros_workspace_path(filepath, lambda, x_lambda);
%genera un file binario che poi verrà utilizzato da un API fornita.

%L'interpolazione è fatta con le spline perché cosi il tutto è smooth. 



