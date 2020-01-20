/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.utilities.XboxController;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    private static RobotContainer robotContainerInstance;
    private DriveTrain mDrive;
    public XboxController mDriverController;
    private CommandScheduler mCommandScheduler;
    private AlignToGoal mAlignToGoal;

    public double mDriveLeftStickX = 0.0, mDriveLeftStickY = 0.0;
    public double mDriveRightStickX = 0.0, mDriveRightStickY = 0.0;

    public double counter = 0.0;

    double joystickLeftX;    
    double joystickLeftY;
    
    double joystickRightX;
    double joystickRightY;

    double leftTrigger;
    double rightTrigger;

  public static RobotContainer getInstance() {
    if(robotContainerInstance == null) {
      robotContainerInstance = new RobotContainer();
    }
    return robotContainerInstance;
  }



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    mDrive = DriveTrain.getInstance();
    mCommandScheduler = CommandScheduler.getInstance();
    mAlignToGoal = new AlignToGoal();
    // Configure the button bindings
    configureButtonBindings();
    // Configure the Commands
    mDrive.setDefaultCommand(
      new RunCommand(() -> mDrive.setSpeedTurn(-1.0 * mDriverController.getDeadbandedLeftYAxis(Constants.XboxController.DEAD_BAND),
        1.0 * mDriverController.getDeadbandedRightXAxis(Constants.XboxController.DEAD_BAND)), mDrive));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    mDriverController = new XboxController(Constants.XboxController.DRIVER_PORT);
    //mDriverController.ButtonA.whileHeld(() -> new AlignToGoal(), mDrive);
    mDriverController.ButtonA.whenHeld(new AlignToGoal());
    //mDriverController.ButtonA.whileHeld(() -> mCommandScheduler.schedule(new AlignToGoal()), mDrive);
    //mDriverController.ButtonA.whileHeld(mAlignToGoal);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
