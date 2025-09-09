function [T, Q] = four_quadrant_propeller(AT,BT,AQ,BQ,omega,Va,D,rho)
    beta = atan2(Va,0.7*omega*D*0.5);
    CT = 0; CQ = 0;
    for k = 0:length(AT)-1
        CT = CT + AT(k+1)*cos(beta*k) + BT(k+1)*sin(beta*k);
        CQ = CQ + AQ(k+1)*cos(beta*k) + BQ(k+1)*sin(beta*k);
    end
    T = (pi*D^2*rho*CT/8.0)*(Va^2 + (0.7*omega*0.5*D)^2);
    Q = (pi*D^3*rho*CQ/8.0)*(Va^2 + (0.7*omega*0.5*D)^2);
end