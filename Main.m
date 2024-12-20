clear all;
close all;
clc;
recuv_tempest;

Va_trim = 21;
h_trim = 1800;

wind_inertial = [0;0;0];

trim_definition = [Va_trim; h_trim];


%%% Use full minimization to determine trim
[trim_variables, fval] = CalculateTrimVariables(trim_definition, aircraft_parameters);
[trim_state, trim_input]= TrimStateAndInput(trim_variables, trim_definition);
[Alon, Blon, Alat, Blat] = AircraftLinearModel(trim_definition, trim_variables,aircraft_parameters);
%% Problem 2a: Yaw Damper State Space & Locus diagram


    N = 10;
for i = 1:N
        figure(i)
kr = linspace(-25,25,N);
hold on
K = - [0 0 kr(i) 0 0 0];
State_Space_Matrix = Alat + Blat(:,2)*K;
[numR, denR] = ss2tf(Alat,Blat(:,2), K, 0);
rlocus(-numR,denR)
hold off
end
K = -[0 0 1.14 0 0 0];
State_Space = Alat + (Blat(:,2))*K;
damp(State_Space)