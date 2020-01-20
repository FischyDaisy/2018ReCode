/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveArcadeMode extends CommandBase {
  private RobotContainer mRobotContainer;
  private DriveTrain mDrive;
  /**
   * Creates a new DriveArcadeMode.
   */
  public DriveArcadeMode() {
    // Use addRequirements() here to declare subsystem dependencies.
    mRobotContainer = RobotContainer.getInstance();
    mDrive = DriveTrain.getInstance();
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize Command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.setSpeedTurn(mRobotContainer.mDriveLeftStickY, mRobotContainer.mDriveRightStickX);
    SmartDashboard.putNumber("LeftStick Y", mRobotContainer.mDriveLeftStickY);
    // System.out.println("LeftStick Y: " + mRobotContainer.mDriveLeftStickY);
    SmartDashboard.putNumber("RigthStick X", mRobotContainer.mDriveRightStickX);
    // System.out.println("RightStick X" + mRobotContainer.mDriveRightStickX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
