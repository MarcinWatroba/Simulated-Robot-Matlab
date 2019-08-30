function Assignment(serPort)

% serPort is the serial port number (for controlling the actual robot).

disp ('==================')
disp ('Program Starting  ')
disp ('------------------')


function [KOut] = lidarPID(inLidarM,inDistance)
   
    %  Gain Values
    KP = 2.5;   %Proportional
    KI = 0.005; %Integral
    KD = 0.7;   %Derivative   
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
    if (KOut > 0.1) 
        KOut = 0.1;
    end
end

function cameraFollow(inCamera)
    if (any(inCamera) && abs(inCamera) > 0.05)
        turnAngle (serPort, .2, (inCamera * 6));
    end
    SetDriveWheelsCreate(serPort, 0.4, 0.4);
    pause (.1); 
end

%Calculate robots X,Y,Heading and distance travelled
function [X, Y, Heading, Dist] = updateOdometry(inX, inY, inHeading, inDist)
    DistRead = DistanceSensorRoomba(serPort); %Find distance travelled since last call to this function
    AngleRead = AngleSensorRoomba(serPort); %Find angle turned since last call to this function.
    Dist = inDist + DistRead; %Update distance travelled.
    Heading = AngleRead + inHeading;
    %Angle = Heading * 180/3.1459
    X = inX + (DistRead*cos(Heading));
    Y = inY + (DistRead*sin(Heading)); 
end

% Clear Matlab's command window
clc;

% Sets forward velocity to 0 using differential system
SetDriveWheelsCreate(serPort, 0, 0);

% Initialise variables

DError = 0; % Difference in actual wall distance from desired wall distance 
ErrorSum = 0;   %Initialize error sum for integral
Heading = 0; % Current heading of the robot
X = 0; % Robot X location
Y = 0; % Robot Y location
    
% Initialize lowest and highest robot locations in the room
LowestY = 10; 
HighestY = -10;
LowestX = 10;
HighestX = -10;
   
% Read the distance (odometry) sensor and initialize distance accumulator
DistRead = DistanceSensorRoomba(serPort);
Dist1 = 0;

% Read the Lidar. It returns a vector of 680 values, spread symmetrically
% for 120 degrees each side of the front.
% Mid-range (element 341) is the front.
% Values above that element measure the Left side, and below that element
% measure the right side.


% Initialize the last following value to the side that has the closest wall
% reading
LidarRes = LidarSensorCreate(serPort);
[LidarM, LidarD] = min(LidarRes(1:681));
DError = LidarM - 0.5;
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


% Wander and follow walls inside the room for 15 meters, during that, save
% the lowest and highest positions in the room, these will be then used to
% find room's centre
while (Dist1 < 15)
    
    %Pause for 0.1 second to provide time for actualisation of the sim
    pause(.1)
    
    % Read Lidar
    LidarRes = LidarSensorCreate(serPort);  %Read lidar for wall detection
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
        Behaviour = 1;  %wandering
    end

    %Call lidarPID function to get PID output for (wall) distance adjustment
    KO = lidarPID(LidarM,0.5);
    
    %If robot heading is above or equal to 6.2... radians (360 degrees) set
    %the heading to what it is minus the 360 degrees.
    if(Heading >= 6.28319)
        Heading = Heading - 6.28319;
    end
    
    %If robot's location is below or above previous lowest and highest
    %locations, update
    if(Y < LowestY)
       LowestY = Y;
    end
    if(Y > HighestY)
        HighestY = Y;
    end
    
    if(X < LowestX)
       LowestX = X;
    end
    if(X > HighestX)
        HighestX = X;
    end
    
    %Calculate the middle of the room based on lowest and highest robot's
    %locations within it
    MiddleY = ((HighestY + LowestY) / 2);
    MiddleX = ((HighestX + LowestX) / 2);

    
    %Update Robots odometry data with the use of updateOdometry function,
    %returns the robot's X, Y, Heading and distance travelled
    [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);
    
    % Adjust wheel speed by using KO, increasing and decreasing speed of
    % individual wheels makes robot turn, this way it's position can be
    % dynamically adjusted using PID controller output based on given setpoint.
    if(Behaviour == 3)  %if following left
        SetDriveWheelsCreate(serPort, 0.2 + KO, 0.2 - KO);
    elseif(Behaviour == 4)  %if following right
        SetDriveWheelsCreate(serPort, 0.2 - KO, 0.2 + KO);
    elseif(Behaviour == 1) %If wandering turn robot at random angle every three seconds        
        SetDriveWheelsCreate(serPort, 0.5, 0.5);  
    end
    

end
    
%Set robot heading to 0, needed for MAngle calculation
Angle = Heading * 180/3.1459;
turnAngle(serPort, .10, -Angle);

%Update odometry
[X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);

%Get robot's distance to the middle of the room
MDist = sqrt((MiddleY - Y)^2 + (MiddleX - X)^2);
%Get the angle robot has to turn to face the middle of the room
MAngle = atan2(MiddleY - Y, MiddleX - X) * 180/3.1459;

%Turn and drive towards the middle
turnAngle (serPort, .10, MAngle);
travelDist(serPort, .2, MDist);
%update odometry
[X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);

%Read Lidar
LidarRes = LidarSensorCreate(serPort);
%These Lidar readings are used to find the rooms exit
LidarX = LidarRes(316);
LidarY = LidarRes(366);
    
%If the readings of LidarX and Y are above or equal to 4, robot is facing
%the exit, until then perform following
while(LidarX < 4 || LidarY < 4)

    pause(.1)
    %turn one degree
    turnAngle (serPort, .10, 1)
    %Update Odometry
    [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);
    %AngleRead = AngleSensorRoomba(serPort)
    %Heading = AngleRead + Heading;
    %Read lidar
    LidarRes = LidarSensorCreate(serPort);
    %Update distances of LidarX and Y
    LidarX = LidarRes(311);
    LidarY = LidarRes(371);
    
end

%When facing the exit, drive straight for three meters to exit the room
travelDist(serPort, .4, 3);
%Update Odometry
[X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);

    
time = 0; %Initialize time for timer used to make robot turn every three seconds   
     
CDistance = 10; %Initialise distance to the beacon
Camera = CameraSensorCreate(serPort);   %Read camera for beacon detection 

%Wander or follow walls until a beacon is found
while (~any(Camera))
    tic
    pause(.1)
    time = toc + time;  %Get time elapsed for timer used in wandering
    
    % Read Lidar
    Camera = CameraSensorCreate(serPort);   %Read camera for beacon detection
    LidarRes = LidarSensorCreate(serPort);  %Read lidar for wall detection
    [LidarM, LidarD] = min(LidarRes(1:681)); %Read the lidar


    
    %If closest wall reading is less than 0.8m away and if not following a
    %beacon then find on which side is the closest wall and set behaviour
    %to either left or right wall following
    if(LidarM < 0.8 && Behaviour ~= 2)  
        if(LidarD < 420 && LidarD > 250) %If the closest wall is in front then follow the wall on previously followed side
            Behaviour = LastFollowing;
        elseif(LidarD > 419)        %If the closest reading is on left
            Behaviour = 3;
        elseif(LidarD < 251)    %If the closest reading is on right
            Behaviour = 4; 
        end
        LastFollowing = Behaviour;  %Save the side that is followed
    elseif(Behaviour ~= 2)  % If not close to a wall and not following a beacon set behaviour to wandering
        Behaviour = 1;
    end

    %Get PID output for wall following
    KO = lidarPID(LidarM,0.5);
   

    
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
    
    %Update odometry
    [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);

   
end

%After finding a beacon, go towards it until the distance to it is less or
%equal to 0.7
while(CDistance > 0.7)
    
    pause(.1)
        
    %Read the offset angle and distance to the beacon
    [Camera, CDistance] = CameraSensorCreate(serPort); 
    %If any camera sees the beacon
    if (any(Camera))
        
        %call cameraFollow function to go towards the beacon
        cameraFollow(Camera);
    else
        %If robot does not see the camera, set the distance to 10 and go
        %forward
        CDistance = 10;
        SetDriveWheelsCreate(serPort, 0.3, 0.3);
    end
    
    %Update odometry
    [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);
        

end
    
    %Find beacon X and Y with the help of odometry
    BX = X + ((CDistance)*cos(Heading + Camera));
    BY = Y + ((CDistance)*sin(Heading + Camera));
    
    %Initalise the variable holding closest achieved distance between robot
    %and beacon
    ClosestBDistance = 10;
    
   %Depending on which side is the beacon, turn accordingly to help start
   %beacon wall following
   if(Camera > 0)  %if following left
        turnAngle (serPort, .25, -50);
   else
        turnAngle (serPort, .25, 50);
   end
   
   %Update odometry
    [X, Y, Heading] = updateOdometry(X, Y, Heading, Dist1);
    
    %Initialise variables 
    Check = 0; %treated as boolean that stops while loop
    Dist1 = 0; %dist will be used to stop robot from circling around the wall


while (Check == 0)
    
    pause(.1)
    
    LidarRes = LidarSensorCreate(serPort);  %Read lidar for wall detection
    
   if(Camera < 0)  %if following right
        LidarM = min(LidarRes(1:350)); %Read the right lidar
   else
        LidarM = min(LidarRes(351:681)); %Read the left lidar
   end
  
   %Get PID for wall following, this time setpoint is 0.3 (closer wall following) 
   KO = lidarPID(LidarM,0.3);
 
   if(Camera > 0)  %if following left
        SetDriveWheelsCreate(serPort, 0.2 + KO, 0.2 - KO);
   else
        SetDriveWheelsCreate(serPort, 0.2 - KO, 0.2 + KO);
   end
    
   %Update odometry
   [X, Y, Heading, Dist1] = updateOdometry(X, Y, Heading, Dist1);
   
   %Measure Robots distance to the beacon
   RobotBeaconLoc = [X,Y;BX,BY];
   BDistance = pdist(RobotBeaconLoc,'euclidean');
     
   %If current distance to the beacon is lower than previous closest, set
   %it as best
   if(BDistance < ClosestBDistance)
      ClosestBDistance = BDistance;
   end
     
   %If robot followed the wall for more than 3 meters, start checking for
   %the difference between current distance and best distance, if it is
   %close enough, stop this while loop
   if(Dist1 > 3)
      Difference = abs(BDistance - ClosestBDistance);
      if(Difference < 0.01)
         Check = 1;
      end
   end
   
    
end
    
   
   if(Camera > 0)  %if following left turn 90 degrees left
        turnAngle (serPort, .2, 90);
   else
        turnAngle (serPort, .2, -90);   %turn 90 degrees right
   end
   
%Initialise bumper
BumpFront = 0;

%while the wall is not bumped go forward
while(BumpFront == 0)
    SetDriveWheelsCreate(serPort, 0.2, 0.2);
    [~, ~, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
end
    %After bumping the wall, reverse and turn around
    travelDist(serPort, .2, -0.4)
    turnAngle (serPort, .2, 180)

    %Initialise the FoundEntrance variable that is treated as a boolean to
    %end the while loop and can also hold the amount of degrees for a turn
    %in turnAngle
    FoundEntrance = 0;
while (FoundEntrance == 0)
    
    pause(.1)
    
    % Read Lidar
    LidarRes = LidarSensorCreate(serPort);  %Read lidar for wall detection
    [LidarM, LidarD] = min(LidarRes(1:681)); %Read the lidar
    
    %Left side lidars used for finding the entrance
    LidarLastLeft = LidarRes(631);
    LidarMiddleLeft = LidarRes(590);
    LidarLeft = LidarRes(610);
    
    %Right side lidars used for finding the entrance
    LidarLastRight = LidarRes(50);
    LidarMiddleRight = LidarRes(91);
    LidarRight = LidarRes(71);

    
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
    
    %Get the PID output
    KO = lidarPID(LidarM,0.5);
    

    
    % Adjust wheel speed by using KOut, increasing and decreasing speed of
    % individual wheels makes robot turn, this way it's position can be
    % dynamically adjusted using PID controller output based on given setpoint.
    if(Behaviour == 3)  %if following left
        SetDriveWheelsCreate(serPort, 0.2 + KO, 0.2 - KO);
    elseif(Behaviour == 4)
        SetDriveWheelsCreate(serPort, 0.2 - KO, 0.2 + KO);
    elseif(Behaviour == 1) %If not following a wall, go straight
        
        SetDriveWheelsCreate(serPort, 0.5, 0.5);

    end
    
    %Various conditions that need to be met to assume that the entrance is
    %to the left of the robot
    if(LidarLeft == 4 || (LidarRight > 0.8 && LidarLeft < LidarMiddleLeft && LidarLeft < LidarLastLeft && Behaviour == 3))
        if(LidarMiddleLeft < 4 && LidarMiddleLeft > 2.8)
            if(LidarLeft == 4 || LidarMiddleLeft > 3.0)
                if(LidarLastLeft < 4 && LidarLastLeft > 2.8)
                    FoundEntrance = 90;
                end
            end
        end
    end

    %Same as above, this time right
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
%Turn towards the entrance at angle specified by FoundEntrance variable (90 or -90)
turnAngle (serPort, .10, FoundEntrance);

%While distance measured by front lidar is more than 1.5m, go forward,
%once the distance is at 1.5 it is assumed that robot is in the middle
%of the room
while(LidarRes(341) > 1.5)
   LidarRes = LidarSensorCreate(serPort); 
   SetDriveWheelsCreate(serPort, 0.3, 0.3);
end

    

% Stop motors
SetDriveWheelsCreate(serPort, 0, 0);
    
    
end        

    
   
    