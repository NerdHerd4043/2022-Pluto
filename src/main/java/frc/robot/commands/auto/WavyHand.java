// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class WavyHand extends CommandBase {
  Drivetrain drivetrain;

  /** Creates a new WavyHand. */
  public WavyHand(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("auto_state", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double fwd = SmartDashboard.getNumber("drive_scalar", 0);
    Double rot = SmartDashboard.getNumber("turn_scalar", 0);

    System.out.println(fwd + " " + rot);

    drivetrain.drive(-fwd, rot, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("auto_state", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
