# 3648-Comp-2022
<h2>Setup</h2>

<ol type="2">
<li>Disable all firewalls and disable all network adapters besides the one that you are using (ethernet or wifi).</li>
<li>Open updated versions of WPILIB VS-Code, Frc Driver Station, and Shuffleboard.</li>
<li>Put a battery in the holder, velcro it in and plug it in to the switch on the right side of the climber. Push the black lever on the switch to turn the robot on. Radio will work after about 2 minutes. Make sure to push the climber above the locking mechanism so you don't break it.</li>
<li>Connect through wifi or ethernet, plug in two controllers, and enable the robot in Teleop Mode and Refer to the controller diagram.</li>
<li>For Autonomous mode set “auton” in Shuffleboard to ‘1’ for one ball auton, ‘2’ for two ball auton,  and ‘3’ for short range 2 ball auton. Auton needs encoders and limelight calibrated to work well. As soon as it is enabled it will start driving.</li>
<li>Space or Enter will emergency stop the robot and you will need to redeploy code to get it to work again.</li>
<li>Code needs to be redeployed any time you make changes, you can deploy the code using (SHIFT+F5).</li>
</ol>

<h2>Controls</h2>
<h3>Driver</h3>
<ul>
  <li>Joystick(1) = Drive forward and backward</li>
  <li>Joystick(4) = Turn left and right</li>
  <li>Button(5) = Intake out & uptake1 down</li>
  <li>Button(6) = Intake in & uptake1 up</li>
  <li>Button(4) = Limelight lineup</li>
  <li>Left Trigger(2) = Climber down (Stops at break)</li>
  <li>Left Trigger(2) + D-pad(up) = Climber down (Full Speed and past limit)</li>
  <li>Right Trigger(3) = Climber up</li>
  <li>Button(7) = Calibrate climber(Make sure the climber is above the IR break)</li>
  <li>Button(8) = Complete climber limit override</li>
</ul>

<h3>Shooter</h3>
<ul>
  <li>Right Trigger(3) = Shoot(Feeds ball into shooter when ball is coverving IR sensor)</li>
  <li>Left Trigger(2) = Manual shooter speed controll</li>
  <li>Button(5) = Intake out & uptake1 out</li>
  <li>Button(6) = Intake in & uptake1 up</li>
  <li>Button(3) = Uptake2 up until ball covers IR sensor</li>
  <li>D-pad(up) = Top goal</li>
  <li>D-pad(down) = Low goal</li>
  <li>D-pad(right) = Top goal(more power)</li>
  <li>D-pad(left) = Limelight generated speed(not accurate(fix?))</li>
  <li>Joystick(1) = Uptake1 up/down</li>
  <li>Joystick(5) = Uptake2 up/down(Overrides IR sensor stop)</li>
</ul>

<h2>Drivers Manual</h2>
<ul>
  <li>Be carefull at full speed, it very easy to tip over</li>
  <li>When climber is locked it can't be unlocked without a human</li>
  <li>If intake is not deployed in auto you must jerk it down</li>
  <li>Best shooting spot is between 170-210 inches from the hoop</li>
</ul>


<h2>Shooter Manual</h2>
<ul>
  <li>The controller will shake when the rpm is high enough for you to shoot</li>
  <li>To achieve the best accuracy use D-pad for shooter speed, x for uptake, and right trigger for shoot</li>
  <li>**WARNING** When using manual shooter speed faster than 0.6 will lead to delamination of the fly wheel **WARNING**</li>
</ul>


