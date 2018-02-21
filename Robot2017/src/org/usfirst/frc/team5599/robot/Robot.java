
/*
	========================================================================================================
		FRC Team 5599 | The Sentinels
		Benjamin N. Cardozo High School
			Team5599.com
	========================================================================================================
*/

/* Useful diagrams for mecanum drive physics
 * https://mililanirobotics.gitbooks.io/frc-electrical-bible/content/Drive_Code/custom_program_mecanum_drive.html 
 */

package org.usfirst.frc.team5599.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.CameraServer;
// import edu.wpi.cscore.AxisCamera;
// import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.PowerDistributionPanel;




public class Robot extends SampleRobot {

	/*
		========================================================================================================
								Initialization
		========================================================================================================
	*/

		// Preferences
	boolean twoStickDrive = true;

	double liftArmSpdOut = 0.8;
	double liftArmSpdIn = 0.4;
	double liftActualBaseSpd = 0.4;
	double gearArmSpd = 0.6;

		// Controllers
	JoystickController driveStickMain;
	JoystickController driveStickLeft;
	XBoxController operatorController;

		// Drive Train
	Spark frontLeft;
	Spark frontRight;
	Spark rearLeft;
	Spark rearRight;

	RobotDrive myRobot;

		// Mechanisms
	Spark liftArm;
	Spark liftActual;

	Spark gearArm;

		// Sensors
	PowerDistributionPanel pdp;
	Ultrasonic rangeFinder;
	ADXRS450_Gyro gyro;

		// Dashboard
	DriverStation driverStation;
	DriverStation.Alliance allianceColor;
	
	/*

	SendableChooser teleOpModes;
	Command chosenTeleOpMode;

	SendableChooser autonomousModes;
	Command chosenAutonomousMethod;
	
	*/
	
	// UsbCamera usbCamera;
	// AxisCamera axisCamera;

	public Robot() {

			// Controllers
		driveStickMain = new JoystickController(0);
		driveStickLeft = new JoystickController(1);

		operatorController = new XBoxController(2);

			// Drive Train
		frontLeft = new Spark(3);
		frontRight = new Spark(2);
		rearLeft = new Spark(5);
		rearRight = new Spark(7);	

		myRobot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

			// Mechanisms
		liftArm = new Spark(1);
		liftActual = new Spark(6);
		gearArm = new Spark(4);

			// Sensors
		pdp = new PowerDistributionPanel(0);

		gyro = new ADXRS450_Gyro();
		gyro.calibrate();

		// usbCamera = CameraServer.getInstance().startAutomaticCapture(0);
		// axisCamera = CameraServer.addAxisCamera().;

		// rangeFinder = new Ultrasonic(1,1);
		
	}

	// Driverstation Stuff

	public void robotInit(){

			// Dashboard
		driverStation = DriverStation.getInstance();
		allianceColor = driverStation.getAlliance();

		getLiftActualSpeed();
		updateDashboard();

		/*
		teleOpModes = new SendableChooser();
		teleOpModes.addDefault("DriveTrain - Mecanum Hybrid", new driveTrainMecanum());
		teleOpModes.addObject("DriveTrain - Tank Drive", new driveTrainTankDrive());
		teleOpModes.addObject("DriveTrain - Mecanum Cartesian", new driveTrainMecanumCartesian(false));
		teleOpModes.addObject("DriveTrain - Mecanum Cartesian [Field Oriented]", new driveTrainMecanumCartesian(true));

		SmartDashboard.putData("DriveTrain Selection", teleOpModes);

		autonomousModes = new SendableChooser();
		autonomousModes.addDefault("Competition - Go Straight To Base Line", new autonomous_GoStraightToPointZone());
		autonomousModes.addObject("Do Nothing", new autonomous_doNothing());
		autonomousModes.addObject("Competition - Left Gear Start", new autonomous_LeftStart());
		autonomousModes.addObject("Competition - Center Gear Start", new autonomous_CenterStart());
		autonomousModes.addObject("Competition - Right Gear Start", new autonomous_RightStart());
		autonomousModes.addObject("Presentation - Do A Square", new autonomous_PresentationSquare());
		autonomousModes.addObject("Presentation - Do A Figure Eight", new autonomous_PresentationFigureEight());

		SmartDashboard.putData("Autonomous Mode Selection", autonomousModes);
		
		*/

		String strAllianceColor;
		int allianceStationLocation = driverStation.getLocation();

		if (allianceColor == DriverStation.Alliance.Blue){
			strAllianceColor = "Blue";
		}
		else if (allianceColor == DriverStation.Alliance.Red){
			strAllianceColor = "Red";
		}
		else {
			strAllianceColor = "Invalid";
		}

		SmartDashboard.putString("Alliance Color", strAllianceColor + " " + allianceStationLocation);
		getLiftActualSpeed();

		CameraServer.getInstance().startAutomaticCapture();

		// Placed under robotInit because it really only needs to run once
		
	}

	public void updateDashboard() {

		double pdp_Temperature = pdp.getTemperature();
		// double pdp_TotalCurrent = pdp.getTotalCurrent();
		double pdp_InputVoltage = pdp.getVoltage();

		double batteryVoltage = driverStation.getBatteryVoltage();

		SmartDashboard.putNumber("Battery Voltage", batteryVoltage);
		SmartDashboard.putNumber("PDP Input Voltage", pdp_InputVoltage);
		// SmartDashboard.putNumber("PDP Total Current", pdp_TotalCurrent);
		SmartDashboard.putNumber("PDP Temperature", pdp_Temperature);

		if (pdp_InputVoltage < 6.5){
			System.out.println("** VOLTAGE BELOW 6.5v | POTENTIAL STICKYFAULT!");
			pdp.clearStickyFaults();
		}

		// This is a method that is called repeatedly

	}

	// Base Methods

	public void operateGearArm(){
		if (operatorController.getAButton()){
			gearArm.set(gearArmSpd);
		}
		else if (operatorController.getBButton()){
			gearArm.set(-gearArmSpd);
		}
		else {
			gearArm.set(0.0);	
		}
	}

	public void operateLiftArm(){
		if (operatorController.getRightBumper()){
			liftArm.set(liftArmSpdIn);
		}
		else if (operatorController.getLeftBumper()){
			liftArm.set(-liftArmSpdOut);
		}
		else {
			liftArm.set(0.0);	
		}
	}

	public double getLiftActualSpeed(){
		// Using the Throttle slider on the driveStickLeft, we can control how much speed the Climber will run at

		double maxAdd = 1 - liftActualBaseSpd;
		double speedAdd = ((driveStickLeft.getThrottle() - 1)/-2)*maxAdd;
		double speedActual = liftActualBaseSpd + speedAdd;

		SmartDashboard.putNumber("Lift Speed", speedActual);

		return speedActual;
	}

	public void operateLiftActual(){
		if (operatorController.getRightTrigger()){
			liftActual.set(getLiftActualSpeed());
			System.out.println("Arm should be running");
		}
		else if (operatorController.getLeftTrigger()){
			liftActual.set(-getLiftActualSpeed());
			System.out.println("Arm should be running BACK");
		}
		else {
			liftActual.set(0.0);	
		}
	}

	/*
		========================================================================================================
								Tele-Operated Code
		========================================================================================================
	*/

		// Potential Drivetrains

	public void driveTrainTankDrive(){

		double stickY = driveStickMain.getJoystickY();
		double stickLeftY = driveStickLeft.getJoystickY();

		myRobot.tankDrive(-stickLeftY, -stickY);

	}

	public void driveTrainMecanumCartesian(boolean useGyro){

		double stickX = driveStickMain.getJoystickX();
		double stickY = driveStickMain.getJoystickY();
		double stickZ = driveStickMain.getJoystickZ();

		double angle = 0.0;

		if (useGyro){
			angle = gyro.getAngle();
		}

		myRobot.mecanumDrive_Cartesian(stickX, stickY, stickZ, angle);

	}

	public void driveTrainMecanum(){

		double stickX = driveStickMain.getJoystickX();
		double stickY = driveStickMain.getJoystickY();
		double stickZ = driveStickMain.getJoystickZ();

		double stickLeftX = driveStickLeft.getJoystickX();
		double stickLeftY = driveStickLeft.getJoystickY();
		
		boolean isStickLeftPushedY = (Math.abs(stickLeftY) > 0.2);

		boolean isStickTwist = (Math.abs(stickZ) > 0.6);
		boolean isStickPushedY = (Math.abs(stickY) > 0.2);
		boolean isStickPushedX = (Math.abs(stickX) > 0.2);

		if (isStickTwist && !twoStickDrive){ // Rotation via Joystick Z Axis Twist
			System.out.println("Turning");
			myRobot.tankDrive(stickZ, -stickZ);
		} 
		else if ((((stickY < 0) && (stickLeftY > 0)) || ((stickY > 0) && (stickLeftY < 0))) && twoStickDrive){ // Rotation via TankDrive w/ Left Stick
			myRobot.tankDrive(-stickLeftY, -stickY);
		}
		else if ((isStickPushedY) && (isStickPushedX)) { // Diagonal Strafing

			// Or use cartesian drive

			if (((stickY > 0) && (stickX > 0)) || ((stickY < 0) && (stickX < 0))) {
				frontRight.set(stickX);
				frontLeft.set(0.0);
				rearRight.set(0.0);
				rearLeft.set(stickX);
				System.out.println("Diagonal 1");
			}
			else if (((stickY > 0) && (stickX < 0)) || ((stickY < 0) && (stickX > 0))) {
				frontRight.set(0.0);
				frontLeft.set(stickX);
				rearRight.set(stickX);
				rearLeft.set(0.0);
				System.out.println("Diagonal 2");
			} else {
				System.out.println("There is no catch-case for this kind of Joystick movement. What did you do?");
			}
		}
		else if (isStickPushedY && !twoStickDrive){ // Forward/Backward Drive
			myRobot.tankDrive(-stickY, -stickY);
			System.out.println("Driving");
		}
		else if ((isStickPushedY || isStickLeftPushedY) && twoStickDrive){
			myRobot.tankDrive(-stickLeftY, -stickY);
			System.out.println("Attempting solo stick drive");
		}
		else if (isStickPushedX) { // Horizontal Strafing
			frontRight.set(stickX);
			frontLeft.set(-stickX);
			rearRight.set(-stickX);
			rearLeft.set(stickX);
			System.out.println("Strafing");
		} else {
			myRobot.tankDrive(0.0, 0.0);
		}
	}

	public void operatorControl() {

		// chosenTeleOpMode = (Command) teleOpModes.getSelected();

		System.out.println("Tele-Op Mode Initiated");
		// System.out.println("DriveTrain Type -> " + chosenTeleOpMode.getName());

		gyro.reset();

		while (isEnabled() && isOperatorControl()){

			// chosenTeleOpMode.start();
			// driveTrainMecanum();
			driveTrainTankDrive();
	
			operateGearArm();
			operateLiftArm();
			operateLiftActual();

			updateDashboard();

			// myRobot.mecanumDrive_Cartesian(driveStickMain.getJoystickX(), driveStickMain.getJoystickX(), driveStickMain.getJoystickZ(), 0.0);

			Timer.delay(0.05);
		}
	}

	/*
		========================================================================================================
								Test Section Code
		========================================================================================================
	*/


	public void test(){

		while (isEnabled() && isTest()){

			System.out.println("Testing Ultrasonic Range Finder");

			myRobot.tankDrive(operatorController.getLeftThumbstickY(), operatorController.getRightThumbstickY());

			if (operatorController.getAButton()){
				System.out.println("Distance: " + rangeFinder.getRangeInches() + " inches");
			}

			updateDashboard();

			Timer.delay(0.1);
		}
	}

	/*
		========================================================================================================
								Autonomous Code
		========================================================================================================
	*/

		// COMPETITION AUTONOMOUS SECTIONS

	public void autonomous_doNothing(){
		System.out.println("Robot not doing anything this autonomous period by choice");
	}

		// Starting Positions
	public void autonomous_LeftStart(){

		autonomous_PlaceGear();
	}

	public void autonomous_RightStart(){
		
		autonomous_PlaceGear();
	}

	public void autonomous_CenterStart(){
		
		autonomous_PlaceGear();
	}

	public void autonomous_GoStraightToPointZone(){

		System.out.println("STARTING Driving forward towards points zone");
		
		double spd = -0.5;

		gyro.reset();
		
		for (int k = 0; k < 18; k++){
			
			System.out.println("Driving forward towards points zone");
			
			rearLeft.set(-spd);
			rearRight.set(spd);
			frontLeft.set(-spd);
			frontRight.set(spd);
			
			if (!isAutonomous()){
				break;
			}
			
			Timer.delay(0.1);
			
		}

		System.out.println("Robot stopping - Should be in points zone");

		rearLeft.set(0.0);
		rearRight.set(0.0);
		frontLeft.set(0.0);
		frontRight.set(0.0);
		
		myRobot.tankDrive(0.0, 0.0);
	}

		// Gear Placement

	public void autonomous_PlaceGear(){

		// Using CameraVision to get the gear peg in, wait, and reverse a bit

		autonomous_PostGearPlacement();
	}

		// Post-Gear Placement

	public void autonomous_PostGearPlacement(){
		// Reverse and navigate towards the points zone

		System.out.println("Reversing and orienting towards points zone");

		while (Math.abs(gyro.getAngle()) < 45){
			System.out.println("Reversing and orienting towards points zone: " + (int)(gyro.getAngle()));
			myRobot.tankDrive(-0.7,-0.7);
			Timer.delay(0.01);
		}

		System.out.println("Driving forward towards points zone");

		for (int i = 0; i < 1000; i++){
			myRobot.tankDrive(0.5,0.5);
			Timer.delay(0.1);
		}

		System.out.println("Robot stopping - Should be in points zone");

		myRobot.tankDrive(0.0, 0.0);

	}

		// PRESENTATION AUTONOMOUS SECTIONS

	public void autonomous_PresentationSquare(){

		System.out.println("Autonomous Presentation Mode - Square");

		gyro.reset();
		
		while (isAutonomous() && isEnabled()){
			
			System.out.println("Autonomous Presentation Mode - Square - Driving Forward");

			for (int tick = 0; tick < 300; tick++){
				if (isEnabled() && isAutonomous()){
					myRobot.tankDrive(0.5, 0.5);
					Timer.delay(.002);
				} else {
					myRobot.tankDrive(0,0);
				}
				
			}
			
			System.out.println("Autonomous Presentation Mode - Square - Holding");
			for (int tick = 0; tick < 500; tick++){
				myRobot.tankDrive(0.0, 0.0);
				Timer.delay(.001);
			}
			
			double cAngle = gyro.getAngle();

			System.out.println("Autonomous Presentation Mode - Square - Turning");
			
			while ((gyro.getAngle() < cAngle + 70) && (gyro.getAngle() < cAngle + 80) && isEnabled() && isAutonomous()){
				myRobot.tankDrive(0.5, -0.5);
				Timer.delay(0.005);
			}

			System.out.println("Autonomous Presentation Mode - Square - Holding");	
			
			for (int tick = 0; tick < 500; tick++){
				myRobot.tankDrive(0.0, 0.0);
				Timer.delay(.001);
			}

			System.out.println("Autonomous Presentation Mode - Square - Cycle Completed");
			
		}
	}

	public void autonomous_PresentationFigureEight(){

		System.out.println("Autonomous Presentation Mode - Figure Eight");

		gyro.reset();

		while (isAutonomous() && isEnabled()) {

			System.out.println("Autonomous Presentation Mode - Figure Eight - Stage 1 Driving");


			double cAngle = gyro.getAngle() + 180;
			
			for (int tick = 0; tick < 1000; tick++){
				myRobot.tankDrive(0.5, 0.55);
				Timer.delay(.001);
			}

			System.out.println("Autonomous Presentation Mode - Figure Eight - Stage 1 Turning");
			
			while ((gyro.getAngle() < cAngle + 170) && (gyro.getAngle() < cAngle + 190) && isEnabled() && isAutonomous()){
				myRobot.tankDrive(0.8, 0.45);
				Timer.delay(0.005);
			}

			System.out.println("Autonomous Presentation Mode - Figure Eight - Stage 2 Driving");
			
			for (int tick = 0; tick < 1000; tick++){
				myRobot.tankDrive(0.55, 0.5);
				Timer.delay(.001);
			}
			System.out.println("Autonomous Presentation Mode - Figure Eight - Stage 2 Turning");

			cAngle = gyro.getAngle() + 180;
			
			while ((gyro.getAngle() > cAngle - 170) && (gyro.getAngle() < cAngle - 190) && isEnabled() && isAutonomous()){
				myRobot.tankDrive(0.45, 0.8);
				Timer.delay(0.005);
			}

			System.out.println("Autonomous Presentation Mode - Figure Eight - Cycle Completed");
			
		}	
	}

	public void autonomous(){

		/*
			Ideally,
				Orient self towards the peg (Depending on starting position)
				Face peg and insert
				Wait a few seconds for the gear to be reeled up (Timed vs Camera detection)
				Reverse out, orient towards point zone, and drive to point zone
				Full stop

		*/

		System.out.println("Autonomous Mode Initiated");

		// chosenAutonomousMethod = (Command) autonomousModes.getSelected();


		/*
			This is a thread that runs alongside the normal autonomous operations to help keep the custom DriverStation updated
		*/

		Thread driverStationTracker = new Thread(() -> {
		 	while (!Thread.interrupted() && isEnabled() && isAutonomous()){
		 		updateDashboard();
		 		Timer.delay(0.1);
		 	}
		 });

		 driverStationTracker.start();
		 
		 autonomous_GoStraightToPointZone();
		 //autonomous_PresentationSquare();
		 // System.out.println("Autonomous Type -> " + chosenAutonomousMode.getName());

		 // chosenAutonomousMethod.start();

		 System.out.println("AUTONOMOUS CODE RUN CONCLUDED \n ---------[5599]--------- ");

	}

	/*
		========================================================================================================
								Disable Code
		========================================================================================================
	*/

		/*
		 * The disable method should go through each and every created motor/servo/etc. and stop them..
		 */

	public void disabled() {
		System.out.println("Disabling Robot");
		myRobot.tankDrive(0.0, 0.0);
		System.out.println("- Drivetrain Disabled");
		gearArm.set(0.0);
		System.out.println("- Gear Arm Disabled");
		liftArm.set(0.0);
		liftActual.set(0.0);
		System.out.println("- Lift Mechanism Disabled");
		System.out.println("Robot Disabled \n ---------[5599]--------- ");
	}

}