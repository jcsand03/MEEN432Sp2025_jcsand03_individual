function [loopscompleted, completetime, offtrack, distancefromcenter, x_track, y_track] = raceStat(x_vals, y_vals, time_vals, track)
    %initalize loops completed and set the car on track to begin
    loopscompleted = 0;
    offtrack = false;

    %make the first straightaways
    straightx1 = linspace(0,track.l_straightaways-1,track.l_straightaways)
    straighty1 = zeros(track.l_straightaways)
    straighty1 = straighty1(1,:,:)

    %make the second straightaway
    straightx2 = linspace(0,track.l_straightaways-1,track.l_straightaways)
    straightx2 = flip(straightx2)
    straighty2 = zeros(track.l_straightaways)
    straighty2 = straighty2 + 400
    straighty2 = straighty2(1,:,:)

    %create the semicircle ends
    %define the ceneter points
    x_center = 900
    y_center = 200

    %calculate arc length
    arc_length = 2*pi*track.radius*(180/360)
    %calculate angles and locations
    theta = linspace(0, pi, arc_length)

    x_semi1 = track.radius * sin(theta) + x_center
    y_semi1 = track.radius * cos(theta) + y_center
    y_semi1 = flip(y_semi1)

    %repeats
    x_center = 0
    y_center = 200

    x_semi2 = (track.radius * sin(theta) + x_center)/-1
    y_semi2 = (track.radius * cos(theta) + y_center)

    %combine all values to make 1 set of x and y values
    x_track = [straightx1, x_semi1, straightx2, x_semi2];
    y_track = [straighty1, y_semi1, straighty2, y_semi2];

    %calcaulte the track offsets
    x_track_outer = zeros(size(x_track));
    y_track_outer = zeros(size(y_track));
    x_track_inner = zeros(size(x_track));
    y_track_inner = zeros(size(y_track));

    for i = 1:length(x_track)-1 
        dx = x_track(i+1) - x_track(i);
        dy = y_track(i+1) - y_track(i);

        %get the normal vector
        normal = [-dy, dx];
        normal = normal / norm(normal);

        %compute upper and inner track offsets
        x_track_outer(i) = x_track(i) + track.width * normal(1);
        y_track_outer(i) = y_track(i) + track.width * normal(2);

        x_vals_inner(i) = x_track(i) - track.width * normal(1);
        y_vals_inner(i) = y_track(i) - track.width * normal(2);
    end

    % handle last points
    x_track_outer(end) = x_track(end);
    y_track_outer(end) = y_track(end);
    x_track_inner(end) = x_track(end);
    y_track_inner(end) = y_track(end);

    % distance from centerline inialized
    distancefromcenter = zeros(size(x_track));

    %check car position
    for i = 2:length(x_vals)
        %see if loop is completed
        if x_vals(i-1) < 0 && x_vals(i) > 0 && y_vals(i) < 1
            loopscompleted = loopscompleted + 1;
        end
        %nearest index to the position then calcualte the distance from the
        %center
        [~, nearest_idx] = min((x_track - x_vals(i)).^2 + (y_track - y_vals(i)).^2);

        distancefromcenter(i) = sqrt((x_vals(i) - x_track(nearest_idx))^2 + (y_vals(i) - y_track(nearest_idx))^2);

        if abs(distancefromcenter(i)) >= track.width/2 - 1 %asummed half of car width is 1 meter
            offtrack = true;
        end
    end

    if loopscompleted > 0
        crossings = find(x_vals(1:end-1) < 0 & x_vals(2:end) >= 0);
        completion_times = time_vals(crossings); % times when each lap finishes
        completetime = diff([0, completion_times(:)']); % time taken for each lap
    else
        completetime = time_vals(end); % if no loops, just final time
    end


    disp(length(x_track))
    disp(length(x_vals))
end
