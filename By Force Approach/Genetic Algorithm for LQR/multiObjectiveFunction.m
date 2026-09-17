function cost = multiObjectiveFunction(K)
assignin('base', 'K', K);
sim("Segway_Robot_Using_LQR_Controller_By_Force.slx");
itae_position = ITAE1(length(ITAE1));
itae_angle = ITAE2(length(ITAE2));
cost = [itae_position itae_angle]
end
