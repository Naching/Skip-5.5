package org.usfirst.frc3467.subsystems.DriveBase.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3467.OI;
import org.usfirst.frc3467.commands.CommandBase;
/**
 *
 */
public class TurnToAngle extends CommandBase {
    private double angle;
    private final double tolerance = 5;

	public TurnToAngle(double angle) {
        requires(drivebase);
		this.setInterruptible(false);
		this.angle = angle;
    }

    protected void initialize() {
    	drivebase.atAngle = false;
    	drivebase.turnToAngle(angle, tolerance);
    }

    protected void execute() {
    }

    protected boolean isFinished() {
    	return drivebase.atAngle;
    }

    protected void end() {
    }

    protected void interrupted() {
    }
}
