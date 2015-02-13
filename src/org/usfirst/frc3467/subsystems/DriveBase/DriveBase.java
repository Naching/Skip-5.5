package org.usfirst.frc3467.subsystems.DriveBase;

import org.usfirst.frc3467.RobotMap;
//import org.usfirst.frc3467.pid.PIDFManager;
import org.usfirst.frc3467.subsystems.DriveBase.commands.DriveMecanum;
import org.usfirst.frc3467.commands.CommandBase;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase extends Subsystem {

	public static final boolean debugging = true;

	private CANTalon CANTalonFL, CANTalonRL, CANTalonFR, CANTalonRR;
	private RobotDrive drive;
	public boolean atAngle = false;

	private static DriveBase instance;

	public static DriveBase getInstance() {
		return instance;
	}

	protected void initDefaultCommand() {
		this.setDefaultCommand(new DriveMecanum());
	}

	public DriveBase() {
		instance = this;

		CANTalon CANTalonFL = new CANTalon(RobotMap.driveTrainCANTalonFL);
		CANTalon CANTalonRL = new CANTalon(RobotMap.driveTrainCANTalonRL);
		CANTalon CANTalonFR = new CANTalon(RobotMap.driveTrainCANTalonFR);
		CANTalon CANTalonRR = new CANTalon(RobotMap.driveTrainCANTalonRR);

		drive = new RobotDrive(CANTalonFL, CANTalonRL, CANTalonFR, CANTalonRR);

		drive.setSafetyEnabled(true);
		drive.setExpiration(0.1);
		drive.setSensitivity(0.5);
		drive.setMaxOutput(1.0);
		drive.setInvertedMotor(MotorType.kFrontLeft, false);
		drive.setInvertedMotor(MotorType.kRearLeft, false);
		drive.setInvertedMotor(MotorType.kFrontRight, true);
		drive.setInvertedMotor(MotorType.kRearRight, true);

	}

	// Set up for arcade driving by PercentVBus
	public void initArcade() {
		CANTalonFL.changeControlMode(ControlMode.PercentVbus);
		CANTalonRL.changeControlMode(ControlMode.PercentVbus);
		CANTalonFR.changeControlMode(ControlMode.PercentVbus);
		CANTalonRR.changeControlMode(ControlMode.PercentVbus);

		drive.setMaxOutput(1.0);

	}

	// Use arcade mode to drive
	public void driveArcade(double speed, double angle) {
		drive.arcadeDrive(speed, angle);
		updateSD();
	}

	// Set up for tank driving by PercentVBus
	public void initTank() {
		CANTalonFL.changeControlMode(ControlMode.PercentVbus);
		CANTalonRL.changeControlMode(ControlMode.PercentVbus);
		CANTalonFR.changeControlMode(ControlMode.PercentVbus);
		CANTalonRR.changeControlMode(ControlMode.PercentVbus);

		drive.setMaxOutput(1.0);

	}

	// Use standard tank drive
	public void driveTank(double left, double right) {
		drive.tankDrive(left, right);
		updateSD();
	}

	// Set up for mecanum driving by Velocity
	public void initMecanum() {
		CANTalonFL.changeControlMode(ControlMode.Speed);
		CANTalonRL.changeControlMode(ControlMode.Speed);
		CANTalonFR.changeControlMode(ControlMode.Speed);
		CANTalonRR.changeControlMode(ControlMode.Speed);

		CANTalonFL.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		CANTalonRL.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		CANTalonFR.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		CANTalonRR.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

		// Determine maximum velocity of our wheels
		drive.setMaxOutput(20.0);

		// TODO: Set PIDs using PIDFManager...
		// CANTalonFL.setPID(0.0, 0.0, 0.0, 0.0, 0, 0, 0);

	}

	// Use mecanum
	public void driveMecanum(double x, double y, double rotation,
			double gyroAngle) {
		drive.mecanumDrive_Cartesian(x, y, rotation, gyroAngle);
		updateSD();
	}

	// does not work for sure, needs testing once we get our hands on a robot
	public void turnToAngle(double angle, double tolerance) {
		final int CW = 0;
		final int CCW = 1;
		int direction = CW;
		double angleInRange;
		atAngle = false;
		boolean wrapAround = false;
		double driveSpeed = 0;
		CANTalonFL.changeControlMode(ControlMode.Speed);
		CANTalonRL.changeControlMode(ControlMode.Speed);
		CANTalonFR.changeControlMode(ControlMode.Speed);
		CANTalonRR.changeControlMode(ControlMode.Speed);
		drive.setMaxOutput(20.0);
		CANTalonFL.set(0);
		CANTalonRL.set(0);
		CANTalonFR.set(0);
		CANTalonRR.set(0);
		angleInRange = (((CommandBase.imu.getYaw() % 360) + 360) % 360);
		if ((angleInRange >= angle && angleInRange - angle <= 180)
				|| (angleInRange < angle && angle - angleInRange > 180)) {
			direction = CCW;
		}
		if ((direction == CCW && angleInRange < angle && angle
				- angleInRange > 180)
				|| (direction == CW && angleInRange > angle && angle
						- angleInRange <= 180)) {
			wrapAround = true;
		}
		if(direction == CW && wrapAround){
			angle = angle + 360;
		}
		if(direction == CCW && wrapAround){
			angle = angle - 360;
		}
		switch (direction) {
		case CW:
			while (!atAngle) {
				driveSpeed = Math.abs(((((CommandBase.imu.getYaw() % 360) + 360) % 360) - angle)/10);
				CANTalonFL.set(driveSpeed);
				CANTalonRL.set(driveSpeed);
				CANTalonFR.set(driveSpeed);
				CANTalonRR.set(driveSpeed);
				if(driveSpeed < 10 * tolerance)
					atAngle = true;
			}
			break;
		case CCW:
			while (!atAngle) {
				driveSpeed = Math.abs(((((CommandBase.imu.getYaw() % 360) + 360) % 360) - angle)/10);
				CANTalonFL.set(-driveSpeed);
				CANTalonRL.set(-driveSpeed);
				CANTalonFR.set(-driveSpeed);
				CANTalonRR.set(-driveSpeed);
				if(driveSpeed < 10 * tolerance)
					atAngle = true;
			}
			break;
		}
	}

	// Refresh Smart Dashboard values
	public void updateSD() {
		if (debugging) {
			// Print data to smart dashboard
			SmartDashboard.putNumber("FL Encoder", CANTalonFL.getEncPosition());
			SmartDashboard.putNumber("RL Encoder", CANTalonRL.getEncPosition());
			SmartDashboard.putNumber("FR Encoder", CANTalonFR.getEncPosition());
			SmartDashboard.putNumber("RR Encoder", CANTalonRR.getEncPosition());

			// TODO: Add more CANTalon outputs...
		}
	}

}
