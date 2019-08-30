# Simulated-Robot-Matlab
A simulated robot using FSM along with reactive and planned behaviour for exiting a room, finding a beacon and returning back to the room.

<h2>Running the AI</h2>
-AI can only be run in the "iRobotCreateSimulatorToolbox". 
-In Matlab, run the "SimulatorGUI" file.
-Then, within the GUI, click on the Load Map button and open the map "cwMap_MG" or "cwMap_MG2".
-Finally, press start.

<h2>How it works</h2>

The AI architecture was designed to be a hybrid of reactive and planned behaviours. The reactive behaviours such as wandering, following walls and homing to beacon are contained within the separate planned tasks that are performed step by step when particular conditions are met.
Each planned task is a while loop that can contain a series of planned and/or reactive behaviours within it. Below is a breakdown of all the tasks and behaviours in the designed architecture.
<ol>
  <li>	Find the middle of the room.
    <ul>
      <li>Calculate odometry.</li>
      <li>Find the middle of the room based on lowest and highest robot’s locations.</li>
      <li>Drive straight (reactive).</li>
      <li>Follow walls on the left (reactive).</li>
      <li>Follow walls on the right (reactive).</li>
  </ul>
 </li>
    <li></li>
    <li></li>
</ol>
2.	Turn and go towards the calculated middle of the room, update odometry.
3.	Find the exit of the room.
a.	Turn.
b.	Calculate odometry.
c.	Check Lidar for frontal readings indicating the position of the exit.
4.	Exit the room and update odometry.
5.	Find the Beacon.
a.	Calculate odometry.
b.	Wander (reactive).
c.	Follow walls on the left (reactive).
d.	Follow walls on the right (reactive).
e.	Check camera for a beacon reading (reactive).
6.	Go towards the beacon.
a.	Calculate odometry.
b.	Read camera for distance and angle offset of the beacon.
c.	Drive towards the beacon.
7.	Calculate odometry and beacon location.
8.	Circle around the beacon.
a.	Follow the wall at close distance.
b.	Calculate current distance away from the beacon.
c.	Compare current distance with the closest calculated distance.
d.	Stop if current distance is close to the closest distance.
9.	Depending on which side is the beacon, turn either 90 degrees left or right.
10.	Go towards the wall.
a.	Drive forward
b.	Check bumpers for collision.
11.	Reverse and turn around.
12.	Look for the room entrance.
a.	Go straight (reactive).
b.	Follow left wall (reactive).
c.	Follow right wall (reactive).
d.	Obtain various Lidar Readings.
e.	Check if Lidar readings indicate the presence of the entrance.
13.	Turn towards the entrance.
14.	Go towards the middle of the entrance.
a.	Check front lidar distance
b.	Drive straight until distance condition met.
15.	Stop the robot.
