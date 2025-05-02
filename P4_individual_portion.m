%track variables
track.radius = 200; % Radius of Curves
track.width = 15; % Width of the Track
track.l_straightaways = 900; % Length of Straightaways

%track variables
Length = 900 %length of the straight away (meters)
Radius = 200 %the radius of the curved end (meters)
Width = 15 %the width of the track (meters)
half_width = 7.5

%make the first straightaways
straightx1 = linspace(0,Length-1,Length)
straighty1 = zeros(Length)
straighty1 = straighty1(1,:,:)

%make the second straightaway
straightx2 = linspace(0,Length-1,Length)
straightx2 = flip(straightx2)
straighty2 = zeros(Length)
straighty2 = straighty2 + 400
straighty2 = straighty2(1,:,:)

%create the semicircle ends
%define the ceneter points
x_center = 900
y_center = 200

%calculate arc length
arc_length = 2*pi*Radius*(180/360)
%calculate angles and locations
theta = linspace(0, pi, arc_length)

x_semi1 = Radius * sin(theta) + x_center
y_semi1 = Radius * cos(theta) + y_center
y_semi1 = flip(y_semi1)

%repeats
x_center = 0
y_center = 200

x_semi2 = (Radius * sin(theta) + x_center)/-1
y_semi2 = (Radius * cos(theta) + y_center)

%add z dimension
% Slope values (rise over run)
slope_up = 10 / 900;
slope_down = 10 / 900;

% Elevation values for the straightaways
z_straight1 = slope_up * straightx1;  % ascending
z_straight2 = slope_down * straightx2 ;  % descending from 10m to 0m

% Elevation values for the semicircles: assume constant height
z_semi1 = ones(1, length(x_semi1)) * 10;  % Top height
z_semi2 = zeros(1, length(x_semi2));      % Bottom height

% Combine all z-values
z_vals = [z_straight1, z_semi1, z_straight2, z_semi2];

%combine all values to make 1 set of x and y values
x_vals = [straightx1, x_semi1, straightx2, x_semi2];
y_vals = [straighty1, y_semi1, straighty2, y_semi2];
track_center = [x_vals, y_vals]

% Compute normal vectors and offset track edges
x_vals_outer = zeros(size(x_vals));
y_vals_outer = zeros(size(y_vals));
x_vals_inner = zeros(size(x_vals));
y_vals_inner = zeros(size(y_vals));

for i = 1:length(x_vals)-1  % Avoid index error at the end
    dx = x_vals(i+1) - x_vals(i);
    dy = y_vals(i+1) - y_vals(i);

    % Compute normal vector (-dy, dx)
    normal = [-dy, dx];
    normal = normal / norm(normal); % Normalize the vector

    % Compute offset for outer and inner track
    x_vals_outer(i) = x_vals(i) + half_width * normal(1);
    y_vals_outer(i) = y_vals(i) + half_width * normal(2);

    x_vals_inner(i) = x_vals(i) - half_width * normal(1);
    y_vals_inner(i) = y_vals(i) - half_width * normal(2);
end

% Handle the last point separately
x_vals_outer(end) = x_vals(end);
y_vals_outer(end) = y_vals(end);
x_vals_inner(end) = x_vals(end);
y_vals_inner(end) = y_vals(end);

%plot track
figure;
hold on
plot3(x_vals, y_vals, z_vals, 'r--')
plot3(x_vals_outer, y_vals_outer, z_vals, 'b')
plot3(x_vals_inner, y_vals_inner, z_vals, 'b')
zlim([-5, 15]);
ylim([-100, 500]);
xlim([-250, 1150]);
axis equal
grid on
view(3)


%create the patch to move on the track
%first define patch dimenstions
car_len = 15
car_width = 7.5
car_z = 2
car_color = [1, 0, 0]
car_moved = false;

%now make the car patch
car_x_dim = [-car_len/2, car_len/2, car_len/2, -car_len/2]
car_y_dim = [-car_width/2, -car_width/2, car_width/2, car_width/2]
car_z_dim = [-car_z/2, -car_z/2, car_z/2, car_z/2]

car_patch = patch(car_x_dim + x_vals(1), car_y_dim + y_vals(1), car_z_dim + z_vals(1), car_color)

%inialize the path
path = animatedline('Color', 'g')

carData.vxd = 17.75

%run simulink
mdl = 'p4_individual_simulink'
load_system(mdl)
set_param(mdl, 'StartTime', '0', 'StopTime', '3600')
out = sim(mdl)

%get data
x_vals_sim = out.X.Data
y_vals_sim = out.Y.Data
Z_vals_sim = out.Z.Data
distance_time = out.X.Time
psi_vals_sim = out.psi.Data
time_vals_sim = out.tout
energy_time = out.MotorEnergy.Time
simulation_energy = out.MotorEnergy.Data

%sum energy usage
total_energy = sum(simulation_energy)

for i = 1:length(x_vals_sim)-1
    % Get z-position (assuming you want to interpolate z_vals_sim from x/y)
    % Approximate z position using nearest neighbor for simplicity:
    [~, idx] = min((x_vals - x_vals_sim(i)).^2 + (y_vals - y_vals_sim(i)).^2);
    z_pos = Z_vals_sim(i);

    % Create rotation matrix
    Rotation_Matrix = [cos(psi_vals_sim(i)), -sin(psi_vals_sim(i));
                       sin(psi_vals_sim(i)),  cos(psi_vals_sim(i))];
    car_rotated = Rotation_Matrix * [car_x_dim; car_y_dim];

    % Update car 3D position
    set(car_patch, 'XData', car_rotated(1, :) + x_vals_sim(i), ...
                   'YData', car_rotated(2, :) + y_vals_sim(i), ...
                   'ZData', car_z_dim + z_pos);

    % Add to 3D path
    addpoints(path, x_vals_sim(i), y_vals_sim(i), z_pos);

    drawnow limitrate;

    if car_moved == false
        if (abs(x_vals_sim(i) - carData.init.X0) > 10)
            car_moved = true;
        end
    end

    pause(0.01);
end

%plot energy usage vs time
figure;
plot(energy_time, simulation_energy)
xlabel("Time (s)")
ylabel("Energy (J)")
title("Energy Consumed vs Time for Loop")

%use raceStats to get the race analytics
[loopscompleted, completetime, offtrack] = raceStat(x_vals_sim, y_vals_sim, distance_time, track)

%show results
disp("Race Statistics");
if loopscompleted > 0
    disp("Laps Completed: " + loopscompleted);
    for i = 1:length(completetime)
        disp("Lap " + i + " Completion Time: " + completetime(i) + " seconds");
    end
else
    disp("The car did not complete a lap");
end


if offtrack
    disp("The car went off the track")
else
    disp("The car stayed on the track")
end

disp("The Total Energy Consumed = " + total_energy+ "J")