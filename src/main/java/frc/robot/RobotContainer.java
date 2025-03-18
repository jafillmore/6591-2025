// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance; 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import frc.robot.subsystems.ClimberSubsystem;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
    Optional<Alliance> ally = DriverStation.getAlliance();
    private String alli = "None! (What's up with that?)";

    UsbCamera camera1;
    VideoSink server;
  
    // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coral = new CoralSubsystem();
  private final ClimberSubsystem m_climb = new ClimberSubsystem();

  // The driver's controllers
  Joystick m_leftJoystick = new Joystick(OIConstants.kLeftControllerPort);
  Joystick m_rightJoystick = new Joystick(OIConstants.kRightControllerPort);
  Joystick m_buttonboard = new Joystick(OIConstants.kButtonBoardPort);
  

 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    



    camera1 = CameraServer.startAutomaticCapture(0);
    
    server = CameraServer.getServer();
    
    // Run configuration options for Pigeon 2 navigation module
    m_robotDrive.pidgeyConfig();

    // Configure the button bindings
    configureButtonBindings();

     //  Configure dashboard
    configureDashboard();
       
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
       // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_leftJoystick.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_leftJoystick.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_rightJoystick.getZ(), OIConstants.kDriveDeadband),
                DriveConstants.driveFieldRelative),
            m_robotDrive)); 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
     //  Set Wheels in an X configuration to prevent movement
    new JoystickButton(m_leftJoystick, OIConstants.kSetXButton)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    //  Zero the gyro to prevent drift
    new JoystickButton(m_leftJoystick, OIConstants.kGyroRestButton)
        .debounce(0.1)   
        .whileTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    //  Toggle Field Centric vs Robot Centric Driving
    new JoystickButton(m_rightJoystick, OIConstants.kFieldRelativeButton)
        .debounce(0.1)   
        .whileTrue(new InstantCommand(
            () -> m_robotDrive.toggleFieldRelative(),
            m_robotDrive));

    
    //  Toggle Coral Info to Shuffleboard
    new JoystickButton(m_buttonboard, OIConstants.kCoralInfoButton)
    .whileTrue(new InstantCommand(
        () -> m_coral.toggleCoralebugInfo(),
        m_coral));

    //  Toggle Climber Info to Shuffleboard
    new JoystickButton(m_buttonboard, OIConstants.kClimberInfoButton)
    .whileTrue(new InstantCommand(
        () -> m_climb.toggleClimberDebugInfo(),
        m_climb));
 
    //  Toggle Drive Info to Shuffleboard
    new JoystickButton(m_leftJoystick, OIConstants.kdriveInfoButton)
    .whileTrue(new InstantCommand(
        () -> m_robotDrive.toggleDriveDebugInfo(),
         m_robotDrive));

    

    //  eject Coral
    new JoystickButton(m_rightJoystick, OIConstants.kDropCoralButton)
    .debounce(0.1)   
    .whileTrue(new InstantCommand(
        () -> m_coral.pinSet(CoralConstants.kPinDown),
        m_coral))
    .whileFalse(new InstantCommand(
        () -> m_coral.pinSet(CoralConstants.kPinUp),
        m_coral));
        
    //  Toggle Extra Info to Shuffleboard
    new JoystickButton(m_leftJoystick, OIConstants.kdriveInfoButton)
        .whileTrue(new InstantCommand(
            () -> m_robotDrive.toggleDriveDebugInfo(),
            m_robotDrive));

    // Load Coral
    new JoystickButton (m_buttonboard,OIConstants.kLoadButton)
        .whileTrue(Commands.parallel(
             new InstantCommand(
            () -> m_coral.setBothELTR(CoralConstants.kElevatorLoad,CoralConstants.ktLoadAngle),
            m_coral)
            ));


    //  Stow Corral and Elevator
    new JoystickButton (m_buttonboard,OIConstants.kStowButon)
    .onTrue(
        new InstantCommand (
        () -> m_coral.setBothELTR(CoralConstants.kElevatorStow, CoralConstants.ktStowAngle),
        m_coral)
        );




    // Elevator Level 1
    new JoystickButton (m_buttonboard,OIConstants.kL1EButton)
    .onTrue    (new InstantCommand(
        () -> m_coral.setElevator(CoralConstants.kElevatorL1),
        m_coral));
    // Elevator Level 2
    new JoystickButton (m_buttonboard,OIConstants.kL2EButton)
    .onTrue(new InstantCommand(
        () -> m_coral.setElevator(CoralConstants.kElevatorL2),
        m_coral));
    // Elevator Level 3
    new JoystickButton (m_buttonboard,OIConstants.kL3EButton)
    .onTrue(new InstantCommand(
        () -> m_coral.setElevator(CoralConstants.kElevatorL3),
        m_coral));
    // Elevator Level 4
    new JoystickButton (m_buttonboard,OIConstants.kL4EButton)
    .onTrue(new InstantCommand(
        () -> m_coral.setElevator(CoralConstants.kElevatorL4),
        m_coral));
    // Stow Trough

    // Trough Level 1
    new JoystickButton (m_buttonboard,OIConstants.kL1TButton)
    .whileTrue(new InstantCommand(
        () -> m_coral.setTrough(CoralConstants.ktL1Angle),
        m_coral));
    // Trough Level 2
    new JoystickButton (m_buttonboard,OIConstants.kL2TButton)
    .whileTrue(new InstantCommand(
        () -> m_coral.setTrough(CoralConstants.ktL2Angle),
        m_coral));
    // Trough Level 3
    new JoystickButton (m_buttonboard,OIConstants.kL3TButton)
    .whileTrue(new InstantCommand(
        () -> m_coral.setTrough(CoralConstants.ktL3Angle),
        m_coral));
    // Trough Level 4
    new JoystickButton (m_buttonboard,OIConstants.kL4TButton)
    .whileTrue(new InstantCommand(
        () -> m_coral.setTrough(CoralConstants.ktL4Angle),
        m_coral));



    // Wrists Out
    new JoystickButton (m_buttonboard,OIConstants.kWristOutButton)
    .whileTrue(Commands.parallel(
        new InstantCommand(
        () -> m_climb.setWrists(ClimberConstants.kleftWristGrab, ClimberConstants.krightWristGrab),
        m_climb)
        
        
        ));
    // Wrist In
    new JoystickButton (m_buttonboard,OIConstants.kWristInButton)
    .whileTrue(Commands.parallel(
        new InstantCommand(
        () -> m_climb.setWrists(ClimberConstants.kleftWristStow, ClimberConstants.krightWristStow),
        m_climb) 
        ));

    //  Arms Up
    new JoystickButton (m_buttonboard,OIConstants.kArmsUpButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setClimber(ClimberConstants.karmsUp),
        m_climb));

    //  Arms Down
    new JoystickButton (m_buttonboard,OIConstants.kArmsDownButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setClimber(ClimberConstants.karmsDown),
        m_climb));










  }

  private void configureDashboard() {
        
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) { alli="Red";}
        if (ally.get() == Alliance.Blue) { alli="Blue";}
    }
  
    SmartDashboard.putString(   "Alliance", alli);
    
        
    
    
    
    
    // Log Shuffleboard events for command initialize, execute, finish, interrupt
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
                Shuffleboard.addEventMarker(
                    "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandExecute(
            command ->
                Shuffleboard.addEventMarker(
                    "Command executed", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Shuffleboard.addEventMarker(
                    "Command finished", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Shuffleboard.addEventMarker(
                    "Command interrupted", command.getName(), EventImportance.kNormal));
     


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Rotate 180deg
        //  List.of (new Rotation2d(90), new Rotation2d(90), 
        //new Pose2d(0,0,new Rotation2d(180),
        // Pass through these two interior waypoints, making an 's' curve path
         List.of(new Translation2d(0.25,0), new Translation2d(0.75, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }


}
