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
DError = 0;

function [KOut] = lidarPID(inLidarM,inDistance)
   
    %  Gain Values
    KP = 2.5;   %Proportional
    KI = 0.005; %Integral
    KD = 0.7;   %Derivative
    ErrorSum = 0;   %Initialize error sum for integral
    
    ErrorMax = 1;   %Max error limit for integral
    DLError = DError; %Save the last error
    DError = inLidarM - inDistance; %Get current error
    ErrorDiff = DLError - DError;   %Get error difference for Derivative
    ErrorSum = ErrorSum + DError;   %Get error sum by adding current error
    
         %Limit error difference value so that it will not be too high when
     %robot looses a wall
     if (ErrorDiff > .1)
         ErrorDiff = .1;
     elseif (ErrorDiff < -.1)
         ErrorDiff = -.1;
     end

     %Make sure error does not surpass given limit in any direction
     if (ErrorSum > ErrorMax)
         ErrorSum = ErrorMax;
     elseif (ErrorSum < -ErrorMax)
         ErrorSum = -ErrorMax;
     end
    
     %Get final output by adding PID values together after multiplying them
     %by gain
    KOut = (KP * DError) + (KI * ErrorSum) + (KD * ErrorDiff);
    
    %Giving a max limit to the output, in this case limiting the wheel
    %speed change to +- 0.1.
    if (KOut > 0.1) KOut = 0.1;
    end
end

function [AngleRead, DistRead] = cameraFollow(inCamera)
    if (any(inCamera) && abs(inCamera) > 0.05)
        turnAngle (serPort, .2, (inCamera * 6));
    end
    SetDriveWheelsCreate(serPort, 0.4, 0.4);
    pause (.1); 
end

function [X, Y, Heading, Dist] = updateOdometry(inX, inY, inHeading, inDist)
    DistRead = DistanceSensorRoomba(serPort);
    AngleRead = AngleSensorRoomba(serPort);
    Dist = inDist + DistRead;
    Heading = AngleRead + inHeading;
    %Angle = Heading * 180/3.1459;
    X = inX + (DistRead*cos(Heading));
    Y = inY + (DistRead*sin(Heading)); 
end

% Sets forward velocity using differential system
SetDriveWheelsCreate(serPort, 0.0, 0.0);

% Initialise variables plus 1st seonr read for each.
    idx1 = 1;

    
    
% Read the distance (odometry) sensor are initialize distance accumulator

    Dist1 = 0;

% Read the Lidar. It returns a vector of 680 values, spread symmetrically
% for 120 degrees each side of the front.
% Mid-range (element 341) is the front.
% Values above that element measure the Left side, and below that element
% measure the right side.


% Initialize the last following value to the closest side
    LidarRes = LidarSensorCreate(serPort);
    [LidarM, LidarD] = min(LidarRes(1:681));
    
    if(LidarD > 340)
        LastFollowing = 3;
    else
        LastFollowing = 4;
    end

% Behaviour 1 = Wandering
% Behaviour 2 = Beacon following
% Behaviour 3 = Following a wall on left
% Behaviour 4 = Following a wall on right
% initialize behaviour to wandering
    Behaviour = 1;
    Heading = 0;
    CDistance = 10;
    X = 0;
    Y = 0;
    
    time = 0; %Initialize time for timer used to make robot turn every three seconds while wandering
    
% Now check the Lidar regularly while the robot moves, to keep the distance
% to the wall in the desired range, and avoiding colliding with a wall both
% on the side and in front.
% Repeat for a given distance...
Camera = CameraSensorCreate(serPort);   %Read camera for beacon detection

% while (CDistance > 0.6)
%     tic
%     pause(.1)
%     time = toc + time;  %Get time elapsed for timer used in wandering
%     
%     % Read Lidar
%     Camera = CameraSensorCreate(serPort);   %Read camera for beacon detection
%     LidarRes = LidarSensorCreate(serPort);  %Read lidar for wall detection
%     [LidarM, LidarD] = min(LidarRes(1:681)); %Read the lidar
% 
%     % Detecting a beacon and going towards it
%     if (any(Camera) && CDistance > 0.6)
%         disp('Homing to beacon');
%         Behaviour = 2;
%         [Camera CDistance] = CameraSensorCreate(serPort)
%         cameraFollow(Camera);
%         [X, Y, Heading, Dist1] =  updateOdometry(X, Y, Heading, Dist1)
%     elseif (Behaviour ~= 1) %If no beacon detected and not wandering then set behaviour to last wall following one
%         Behaviour = LastFollowing;
%     end
%     
%     %If closest wall reading is less than 0.8m away and if not following a
%     %beacon then find on which side is the closest wall and set behaviour
%     %to either left or right wall following
%     if(LidarM < 0.8 && Behaviour ~= 2)  
%         if(LidarD < 420 && LidarD > 250) %If the closest wall is in front then follow the wall on previously followed side
%             Behaviour = LastFollowing;
%         elseif(LidarD > 419)        %If the closest reading is on left
%             Behaviour = 3;
%         elseif(LidarD < 251)    %If the closest reading is on right
%             Behaviour = 4; 
%         end
%         LastFollowing = Behaviour;  %Save the side that is followed
%     elseif(Behaviour ~= 2)  % If not close to a wall and not following a beacon set behaviour to wandering
%         Behaviour = 1;
%     end
% 
% 
%     KO = lidarPID(LidarM,0.5);
%    
% 
%     
%     % Adjust wheel speed by using KOut, increasing and decreasing speed of
%     % individual wheels makes robot turn, this way it's position can be
%     % dynamically adjusted using PID controller output based on given setpoint.
%     if(Behaviour == 3)  %if following left
%         SetDriveWheelsCreate(serPort, 0.2 + KO, 0.2 - KO);
%     elseif(Behaviour == 4)
%         SetDriveWheelsCreate(serPort, 0.2 - KO, 0.2 + KO);
%     elseif(Behaviour == 1) %If wandering turn robot at random angle every three seconds
%         
%         SetDriveWheelsCreate(serPort, 0.5, 0.5);
%         
%         if (time > 3)   %If timer is above 3 seconds
%         r = randi([-90 -30],1,1);    %find random number for an angle
%         turnAngle (serPort, .5, r); %turn at random angle
%         time = 0;   %Reset timer
%         end       
%     end
%     
%      % Read Odometry sensor and accumulate distance measured.
%     [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);
% 
%    
% end
%     
%     BX = X + ((CDistance)*cos(Heading + Camera));
%     BY = Y + ((CDistance)*sin(Heading + Camera));
%     ClosestBDistance = 10;
%     BestX = 0;
%     BestY = 0;
%    if(Camera > 0)  %if following left
%         turnAngle (serPort, .25, -50);
%    else
%         turnAngle (serPort, .25, 50);
%    end
%     [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1)
%     Check = 0;
%     Dist1 = 0;
% 
% while (Check == 0)
%     
%     pause(.1)
%     
%     LidarRes = LidarSensorCreate(serPort);  %Read lidar for wall detection
%     
%    if(Camera < 0)  %if following left
%         [LidarM, LidarD] = min(LidarRes(1:350)) %Read the lidar
%    else
%         [LidarM, LidarD] = min(LidarRes(351:681)) %Read the lidar
%    end
%   
%     
%    KO = lidarPID(LidarM,0.3);
%  
%    if(Camera > 0)  %if following left
%         SetDriveWheelsCreate(serPort, 0.2 + KO, 0.2 - KO);
%    else
%         SetDriveWheelsCreate(serPort, 0.2 - KO, 0.2 + KO);
%    end
%     
%    [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1)
%     
%    RobotBeaconLoc = [X,Y;BX,BY]
%    BDistance = pdist(RobotBeaconLoc,'euclidean')
%      
%    if(BDistance < ClosestBDistance)
%       ClosestBDistance = BDistance;
%       BestX = X;
%       BestY = Y;
%    end
%      
%    if(Dist1 > 4)
%       Difference = abs(BDistance - ClosestBDistance)
%       if(Difference < 0.005)
%          Check = 1;
%       end
%    end
%     
% end
% 
%    if(Camera > 0)  %if following left
%         turnAngle (serPort, .2, 90);
%    else
%         turnAngle (serPort, .2, -90);
%    end
% BumpFront = 0;
% 
% while(BumpFront == 0)
%     SetDriveWheelsCreate(serPort, 0.2, 0.2);
%     [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster BumpFront]= BumpsWheelDropsSensorsRoomba(serPort)
% end
% 
%     travelDist(serPort, .2, -1)
%     turnAngle (serPort, .2, 180)

    RoomExit = [-4, 0]
    RoomMiddle = [0, 0]
    Y = 0;
    X = 2;
    Heading = 3.14159;
    EntranceDist = sqrt((RoomExit(2) - Y)^2 + (RoomExit(1) - X)^2);
    FoundEntrance = 0;
while (FoundEntrance == 0)
    tic
    pause(.1)
    time = toc + time;  %Get time elapsed for timer used in wandering
    
    % Read Lidar
    LidarRes = LidarSensorCreate(serPort);  %Read lidar for wall detection
    LidarLastLeft = LidarRes(631);
    LidarMiddleLeft = LidarRes(590);
    LidarLeft = LidarRes(610);
    
    LidarLastRight = LidarRes(50);
    LidarMiddleRight = LidarRes(91);
    LidarRight = LidarRes(71);
    [LidarM, LidarD] = min(LidarRes(1:681)); %Read the lidar

    
    %If closest wall reading is less than 0.8m away then find on which side is the closest wall and set behaviour
    %to either left or right wall following
    if(LidarM < 0.8)  
        if(LidarD < 420 && LidarD > 250) %If the closest wall is in front then follow the wall on previously followed side
            Behaviour = LastFollowing;
        elseif(LidarD > 419)        %If the closest reading is on left
            Behaviour = 3;
        elseif(LidarD < 251)    %If the closest reading is on right
            Behaviour = 4; 
        end
        LastFollowing = Behaviour;  %Save the side that is followed
    else
        Behaviour = 1;
    end

    KO = lidarPID(LidarM,0.5);
    
    % Read Odometry sensor and accumulate distance measured.

    
    % Adjust wheel speed by using KOut, increasing and decreasing speed of
    % individual wheels makes robot turn, this way it's position can be
    % dynamically adjusted using PID controller output based on given setpoint.
    if(Behaviour == 3)  %if following left
        SetDriveWheelsCreate(serPort, 0.2 + KO, 0.2 - KO);
    elseif(Behaviour == 4)
        SetDriveWheelsCreate(serPort, 0.2 - KO, 0.2 + KO);
    elseif(Behaviour == 1) %If wandering turn robot at random angle every three seconds
        
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        
        if (time > 3)   %If timer is above 3 seconds
        r = randi([-90 -30],1,1);    %find random number for an angle
        turnAngle (serPort, .5, r); %turn at random angle
        time = 0;   %Reset timer
        end       
    end
    
    [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);
    

        if(LidarLeft == 4 || (LidarRight > 0.8 && LidarLeft < LidarMiddleLeft && LidarLeft < LidarLastLeft && Behaviour == 3))
            if(LidarMiddleLeft < 4 && LidarMiddleLeft > 2.8)
                if(LidarLeft == 4 || LidarMiddleLeft > 3.0)
                    if(LidarLastLeft < 4 && LidarLastLeft > 2.8)
                        FoundEntrance = 90;
                    end
                end
            end
        end
        
        if(LidarRight == 4 || (LidarRight > 0.8 && LidarRight < LidarMiddleRight && LidarRight < LidarLastRight && Behaviour == 4))
            if(LidarMiddleRight < 4 && LidarMiddleRight > 2.8)
                if(LidarRight == 4 || LidarMiddleRight > 3.0)
                    if(LidarLastRight < 4 && LidarLastRight > 2.8)
                        FoundEntrance = -90;
                    end
                end
            end
        end



    
end
   turnAngle (serPort, .10, FoundEntrance);
while(LidarRes(341) > 1.5)
   LidarRes = LidarSensorCreate(serPort); 
   SetDriveWheelsCreate(serPort, 0.3, 0.3);
end


%     [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1)
%     
%     Angle = Heading * 180/3.1459;
%     turnAngle (serPort, .10, -Angle);
%     [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);
% 
% 
%     MDist = sqrt((RoomExit(2) - Y)^2 + (RoomExit(1) - X)^2);
%     MAngle = atan2(RoomExit(2) - Y, RoomExit(1) - X) * 180/3.1459;
%      
%     turnAngle (serPort, .10, MAngle);
%     travelDist(serPort, .2, MDist);
%     [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1)
%     
%     Angle = Heading * 180/3.1459;
%     turnAngle (serPort, .10, -Angle);
%     [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);
% 
% 
%     MDist = sqrt((RoomMiddle(2) - Y)^2 + (RoomMiddle(1) - X)^2);
%     MAngle = atan2(RoomMiddle(2) - Y, RoomMiddle(1) - X) * 180/3.1459;
%      
%     turnAngle (serPort, .10, MAngle);
%     travelDist(serPort, .2, MDist);
%     [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1)

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    
    
end    