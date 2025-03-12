// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
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
    private String alli = "None! (WTF?)";

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
  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Define Autonomous Commands
    private final Command m_redAuto1 = Autos.redAuto1(m_robotDrive);
    private final Command m_redAuto2 = Autos.redAuto2(m_robotDrive);
    private final Command m_redAuto3 = Autos.redAuto3(m_robotDrive);
    private final Command m_blueAuto1 = Autos.blueAuto1(m_robotDrive);
    private final Command m_blueAuto2 = Autos.blueAuto2(m_robotDrive);
    private final Command m_blueAuto3 = Autos.blueAuto3(m_robotDrive);

 
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
    
    
    // Add commands to the autonomous command chooser
    if (ally.get() == Alliance.Red){
        m_chooser.setDefaultOption("Red 1", m_redAuto1);
        m_chooser.addOption("Red 2", m_redAuto2);
        m_chooser.addOption("Red 3", m_redAuto3);
    }

    if (ally.get() == Alliance.Blue){
        m_chooser.setDefaultOption("Blue 1", m_blueAuto1);
        m_chooser.addOption("Blue 2", m_blueAuto2);
        m_chooser.addOption("Blue 3", m_blueAuto3);
    }   

    
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
    new JoystickButton(m_leftJoystick, OIConstants.kdriveDebugDataButton)
        .whileTrue(new InstantCommand(
            () -> m_robotDrive.toggleDriveDebugInfo(),
            m_robotDrive));

    // Load Coral
    new JoystickButton (m_buttonboard,OIConstants.kLoadButton)
        .whileTrue(Commands.parallel(
             new InstantCommand(
            () -> m_coral.setTrough(CoralConstants.ktLoadAngle),
            m_coral),
            new InstantCommand (
            () -> m_coral.setElevator(CoralConstants.kElevatorLoad),
            m_coral)));


    //  Stow Corral and Elevator
    new JoystickButton (m_buttonboard,OIConstants.kStowButon)
    .whileTrue(Commands.parallel(
         new InstantCommand(
        () -> m_coral.setTrough(CoralConstants.ktStowAngle),
        m_coral),
        new InstantCommand (
        () -> m_coral.setElevator(CoralConstants.kElevatorStow),
        m_coral)));




    // Elevator Level 1
    new JoystickButton (m_buttonboard,OIConstants.kL1EButton)
    .whileTrue(new InstantCommand(
        () -> m_coral.setElevator(CoralConstants.kElevatorL1),
        m_coral));
    // Elevator Level 2
    new JoystickButton (m_buttonboard,OIConstants.kL2EButton)
    .whileTrue(new InstantCommand(
        () -> m_coral.setElevator(CoralConstants.kElevatorL2),
        m_coral));
    // Elevator Level 3
    new JoystickButton (m_buttonboard,OIConstants.kL3EButton)
    .whileTrue(new InstantCommand(
        () -> m_coral.setElevator(CoralConstants.kElevatorL3),
        m_coral));
    // Elevator Level 4
    new JoystickButton (m_buttonboard,OIConstants.kL4EButton)
    .whileTrue(new InstantCommand(
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



    // Wrist Out
    new JoystickButton (m_buttonboard,OIConstants.kWristOutButton)
    .whileTrue(Commands.parallel(
        new InstantCommand(
        () -> m_climb.setLeftWrist(ClimberConstants.kleftWristGrab),
        m_climb), 
        new InstantCommand(
        () -> m_climb.setRightWrist(ClimberConstants.krightWristGrab),
        m_climb)));
    // Wrist In
    new JoystickButton (m_buttonboard,OIConstants.kWristInButton)
    .whileTrue(Commands.parallel(
        new InstantCommand(
        () -> m_climb.setLeftWrist(ClimberConstants.kleftWristStow),
        m_climb), 
        new InstantCommand(
        () -> m_climb.setRightWrist(ClimberConstants.krightWristStow),
        m_climb)));

    //  Arms Up
    new JoystickButton (m_buttonboard,OIConstants.kArmStowButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setRightWrist(ClimberConstants.karmsUp),
        m_climb));

    //  Arms Down
    new JoystickButton (m_buttonboard,OIConstants.kArmClimbButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setRightWrist(ClimberConstants.karmsDown),
        m_climb));










  }

  private void configureDashboard() {
        
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) { alli="Red";}
        if (ally.get() == Alliance.Blue) { alli="Blue";}
    }
  
    SmartDashboard.putString(   "Alliance", alli);
        
    
    
    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    // Put subsystems to dashboard.
    //Shuffleboard.getTab("Drivetrain").add(m_robotDrive);
    
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
    return m_chooser.getSelected();
  }


}
