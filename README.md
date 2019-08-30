# Simulated-Robot-Matlab
A simulated robot using FSM along with reactive and planned behaviour for exiting a room, finding a beacon and returning back to the room.

<h2>Running the AI</h2>
-AI can only be run in the "iRobotCreateSimulatorToolbox". 
-In Matlab, run the "SimulatorGUI" file.
-Then, within the GUI, click on the Load Map button and open the map "cwMap_MG" or "cwMap_MG2".
-Finally, press start.

<h2>Screenshot showing robot's path</h2>
<img src="https://github.com/marcin388/Simulated-Robot-Matlab/blob/master/path-screen.png">
![Screenshot Missing](https://github.com/marcin388/Simulated-Robot-Matlab/blob/master/path-screen.png)

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
 <li>Turn and go towards the calculated middle of the room, update odometry.</li>
 <li>Find the exit of the room.
   <ul>
     <li>Turn.</li>
     <li>Calculate odometry.</li>
     <li>Check Lidar for frontal readings indicating the position of the exit.</li>
   </ul>
 </li>
 <li>Exit the room and update odometry.</li>
 <li>Find the Beacon.
   <ul>
     <li>Calculate odometry.</li>
     <li>Wander (reactive).</li>
     <li>Follow walls on the left (reactive).</li>
     <li>Follow walls on the right (reactive).</li>
     <li>Check camera for a beacon reading (reactive).</li>
   </ul>
 </li>
 <li>Go towards the beacon.
   <ul>
     <li>Calculate odometry.</li>
     <li>Read camera for distance and angle offset of the beacon.</li>
     <li>Drive towards the beacon.</li>
   </ul>
 </li>
 <li>Calculate odometry and beacon location.</li>
 <li>Circle around the beacon.
   <ul>
     <li>Follow the wall at close distance.</li>
     <li>Calculate current distance away from the beacon.</li>
     <li>Compare current distance with the closest calculated distance.</li>
     <li>Stop if current distance is close to the closest distance.</li>
   </ul>
 </li>
 <li>Depending on which side is the beacon, turn either 90 degrees left or right.</li>
 <li>Go towards the wall.
   <ul>
     <li>Drive forward</li>
     <li>Check bumpers for collision.</li>
   </ul> 
 </li>
 <li>Reverse and turn around.</li>
 <li>Look for the room entrance.
   <ul>
     <li>Go straight (reactive).</li>
     <li>Follow left wall (reactive).</li>
     <li>Follow right wall (reactive).</li>
     <li>Obtain various Lidar Readings.</li>
     <li>Check if Lidar readings indicate the presence of the entrance.</li>
   </ul> 
 </li>
 <li>Turn towards the entrance.</li>
 <li>Go towards the middle of the entrance.
   <ul>
     <li>Check front lidar distance</li>
     <li>Drive straight until distance condition met.</li>
   </ul>
 </li>
 <li>Stop the robot.</li>
