function Lab6Prog_eg1(serPort)
% Robot moves along a path, part blindly, part sensing.
% serPort is the serial port number (for controlling the actual robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp ('==================')
disp ('Program Starting  ')
disp ('------------------')

%  Use a command like this one to plot the trajectory followed from the
%  saved data (double-click in your saved data file to load it into the
%  datahistory workspace). Check the size and adjuts the index accordingly.
%
%  plot(cell2mat(datahistory(1:2602,2)),cell2mat(datahistory(1:2602,3)));


% Sets forward velocity using differential system
SetDriveWheelsCreate(serPort, 0.5, 0.5);

% Initialise variables plus 1st seonr read for each.
    idx1 = 1;
      
% Read the distance (odometry) sensor are initialize distance accumulator
    DistRead = DistanceSensorRoomba(serPort);
    Dist1 = 0;



time = 0; %Initialize time for timer used to make robot turn every three seconds while wandering
    

while (Dist1 < 30)
    tic
    pause(.1)
    time = toc + time  %Get time elapsed for timer used in wandering
    
    % Read Lidar
    LidarRes = LidarSensorCreate(serPort);  %Read lidar for wall detection
    [LidarM, LidarD] = min(LidarRes(1:681)); %Read the lidar


    
    %If the closest reading is less than 0.5m away
    if(LidarM < 0.5)  
        if(LidarD < 345 && LidarD > 340) %If the closest wall is in front left then make a sharp turn to the right
            turnAngle(serPort, 5, -90);
        elseif(LidarD < 341 && LidarD > 335) %If the closest wall is in front right then make a sharp turn to the left
            turnAngle(serPort, 5, 90);
        elseif(LidarD > 344)        %If the closest reading is on left then turn away slightly
            turnAngle(serPort, 5, -20);
        elseif(LidarD < 336)    %If the closest reading is on right then turn away slightly
            turnAngle(serPort, 5, 20);
        end
    end

    
    % Read Odometry sensor and accumulate distance measured.
    DistRead = DistanceSensorRoomba(serPort);
    Dist1 = Dist1 + DistRead;
        
    SetDriveWheelsCreate(serPort, 0.5, 0.5);
        
    if (time > 3)   %If timer is above 3 seconds
        r = randi([-90 90],1,1);    %find random number for an angle
        turnAngle (serPort, .5, r); %turn at random angle
        time = 0;   %Reset timer
    end       

end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    