function Rover = smartalgoselect(Rover)
Rover.u_sur_u = Rover.u_sur/norm(Rover.u_sur);
rot_rad = -3*pi/4;
u1_sur_u = transpose([cos(rot_rad) -sin(rot_rad);sin(rot_rad) cos(rot_rad)]*transpose(Rover.u_sur_u));
u1_sur = 5*u1_sur_u;
Rover.theta_ref = 0;
int_ind =0;
for i = 0:0.5:270
    s = Rover.pos_curr + u1_sur;
    for j= 1: Rover.Obstacles.Number
        int_ind =0;
        B = [Rover.pos_curr(1),s(1)];
        C = [Rover.pos_curr(2),s(2)];
        [xi,yi] = polyxpoly(B,C,Rover.Obstacles.X(j,:), Rover.Obstacles.Y(j,:));
        int_ind = int_ind + sum([xi,yi]);
        coincide_ind = sum(int_ind);
        if coincide_ind > 0
            Rover.theta_ref = Rover.theta_ref +0.5;
             break;
        end
                
    end
    u1_sur = transpose([cos(pi/180) -sin(pi/180);sin(pi/180) cos(pi/180)]*transpose(u1_sur));
end
end