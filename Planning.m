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
    DError = 0;
    
    %  Gain Values
    KP = 2.5;   %Proportional
    KI = 0.005; %Integral
    KD = 0.7;   %Derivative
    ErrorSum = 0;   %Initialize error sum for integral
    ErrorMax = 1;   %Max error limit for integral
    
ReachedBeacon = 0;  %used to determine if robot reached the beacon or not

SeeCamera = 0; 

%First, go towards a beacon, once it is reached the robot will start following a wall    
while (ReachedBeacon == 0)
     Camera = CameraSensorCreate(serPort);   %Read camera for beacon detection
     if (any(Camera))
        disp('Homing to beacon');
        SeeCamera = 1;  %Robot sees the camera
        Camera = CameraSensorCreate(serPort);
        if (any(Camera) && abs(Camera) > 0.05)
            turnAngle (serPort, .2, (Camera * 6));
        end
            SetDriveWheelsCreate(serPort, 0.4, 0.4);
            pause (.1);
     elseif(~any(Camera) && SeeCamera == 1) %If robot saw a camera but no longer does it can be assumed robot reached the beacon and went past it
         ReachedBeacon = 1; %Reached beacon, therefore follow a wall
     end
end

% Read the Lidar. It returns a vector of 680 values, spread symmetrically
% for 120 degrees each side of the front.
% Mid-range (element 341) is the front.
% Values above that element measure the Left side, and below that element
% measure the right side.

% Behaviour 3 = Following a wall on left
% Behaviour 4 = Following a wall on right
LidarRes = LidarSensorCreate(serPort);
[LidarM, LidarD] = min(LidarRes(1:681));
DError = LidarM - 0.5;
% Initialize the last following value to the closest side
if(LidarD > 340)
   LastFollowing = 3;
else
   LastFollowing = 4;
end

    
% Now check the Lidar regularly while the robot moves, to keep the distance
% to the wall in the desired range, and avoiding colliding with a wall both
% on the side and in front.
% Repeat for a given distance...    
while (ReachedBeacon == 1)
    pause(.1)
    
    % Read Lidar
    LidarRes = LidarSensorCreate(serPort);  %Read lidar for wall detection
    LastLidarD = LidarD;    %Save last lidar element reading
    [LidarM, LidarD] = min(LidarRes(1:681)); %Read the lidar

    % Detecting a beacon and going towards it

    
    %If closest wall reading is less than 0.8m away then find on which side is the closest wall and set behaviour
    %to either left or right wall following
    if(LidarM < 0.8)  
        if(LidarD < 420 && LidarD > 250) %If the closest wall is in front range then follow the wall on previously followed side
            Behaviour = LastFollowing;
        elseif(LidarD > 419)        %If the closest reading is on left then follow it
            Behaviour = 3;
        elseif(LidarD < 251)    %If the closest reading is on right then follow it
            Behaviour = 4; 
        end
        LastFollowing = Behaviour;  %Save the side that is followed
    end

    DLError = DError; %Save the last error
    DError = LidarM - 0.5; %Get current error
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
    
    
    % Adjust wheel speed by using KOut, increasing and decreasing speed of
    % individual wheels makes robot turn, this way it's position can be
    % dynamically adjusted using PID controller output based on given setpoint.
    if(Behaviour == 3)  %if following left
        SetDriveWheelsCreate(serPort, 0.2 + KOut, 0.2 - KOut);
    elseif(Behaviour == 4)  %If following right
        SetDriveWheelsCreate(serPort, 0.2 - KOut, 0.2 + KOut);     
    end

end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    