// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
    Optional<Alliance> ally = DriverStation.getAlliance();
    private String alli = "None! (What's up with that?)";

    //UsbCamera camera1;
    //VideoSink server;
    
  
    // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coral = new CoralSubsystem();
  private final ClimberSubsystem m_climb = new ClimberSubsystem();
  private final AutoFactory autoFactory;
  
  //private final AutoChooser autoChooser;

  private final Timer timer = new Timer();
  String autonStatus = "Not Trying Anything";


  // Loads a swerve trajectory, alternatively use DifferentialSample if the robot is tank drive
  //private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("leftWallToReef");
  //private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("leftWallOutOfTheWay");
  //private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("rightWallToReef");
  //private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("rightWallOutOfTheWay");



  // The driver's controllers
  Joystick m_leftJoystick = new Joystick(OIConstants.kLeftControllerPort);
  Joystick m_rightJoystick = new Joystick(OIConstants.kRightControllerPort);
  Joystick m_buttonboard = new Joystick(OIConstants.kButtonBoardPort);
  

 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    autoFactory = new AutoFactory(
           m_robotDrive::getPose, // A function that returns the current robot pose
           m_robotDrive::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
           m_robotDrive::followTrajectory, // The drive subsystem trajectory follower 
            false, // If alliance flipping should be enabled 
            m_robotDrive // The drive subsystem
        );

        
        






        CameraServer.startAutomaticCapture();
        //camera.setFPS (30);



    //camera1 = CameraServer.startAutomaticCapture(0);
    
    //server = CameraServer.getServer();

    
    
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
    new JoystickButton(m_buttonboard, OIConstants.kdriveInfoButton)
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
        () -> m_coral.setTrough(CoralConstants.ktAlgeaAngle),
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


    // Wrists Out Manual Control
    new JoystickButton (m_buttonboard,OIConstants.kWristOutButton)
    .onTrue(new InstantCommand(
        () -> m_climb.wristsIn(ClimberConstants.kwristOutPower),
        m_climb))
    .onFalse(new InstantCommand(
        () -> m_climb.wristsIn(0.0),
        m_climb));
    
    
    
    // Wrist In
    new JoystickButton (m_buttonboard,OIConstants.kWristInButton)
    .onTrue(new InstantCommand(
        () -> m_climb.wristsIn(ClimberConstants.kwristInPower),
        m_climb))
    .onFalse(new InstantCommand(
        () -> m_climb.wristsIn(0.0),
        m_climb));



    // Reset Wrist Encoders
    new JoystickButton (m_buttonboard,OIConstants.kresetWristEncodersButton)
    .whileTrue(new InstantCommand(
        () -> m_climb.resetWristEncoders(),
        m_climb));

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
    SmartDashboard.putString("Attempted Auton", autonStatus);
    // Create the auto chooser
    AutoChooser autoChooser = new AutoChooser();

    // Add options to the chooser
    autoChooser.addCmd("Left out of the way", this::leftOutOfTheWayCommand);
    autoChooser.addCmd("Left to Reef", this::leftToReefCommand);
    autoChooser.addCmd("Right to Reef", this::rightToReefCommand);
    autoChooser.addCmd("Right out of the way", this::rightOutOfTheWayCommand);

    // Put the auto chooser on the dashboard
    SmartDashboard.putData("Options",autoChooser);


    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    
    
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


   ///////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initiation items for Auton
    /// ///////////////////////////////////////////////////////////////////////////////////////////////////

    public void autoInit() {
       /* 
        if (trajectory.isPresent()) {
            // Get the initial pose of the trajectory
            Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());
            SmartDashboard.putBoolean(   "Traj Loaded",trajectory.isPresent());

            if (initialPose.isPresent()) {
                // Reset odometry to the start of the trajectory
                m_robotDrive.resetOdometry(initialPose.get());
                SmartDashboard.putBoolean(   "Initial Pose",initialPose.isPresent());

            }
        }

        // Reset and start the timer when the autonomous period begins
        timer.restart();
        SmartDashboard.putNumber("Time", timer.get());
        */

    }


    ///////////////////////////////////////////////////////////////////////////////
    private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }
    ///////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    //////////////////////////////////////////////////////////////////////////
    public Command getAutonomousCommand() {
       

        return null; //autoChooser.selectedCommand();
    }

    /////////////////////////////////////////////////////////////////////////
    /// Auto Routine to move to the reef, clear algae and score coral/
    /// /////////////////////////////////////////////////////////////////////

    

    public AutoRoutine scroreOnReefLeftAuto() {
        autonStatus = "Trajactory with Scoring";
        SmartDashboard.putString("Attempting", autonStatus);

        AutoRoutine routine = autoFactory.newRoutine("LeftWallStartScoreOnReef");

        // Load the routine's trajectories
        AutoTrajectory scoreOnReefTraj = routine.trajectory("startonwall");

        // When the routine begins, reset odometry and start the first trajectory (1)
        scroreOnReefLeftAuto().active().onTrue(
            Commands.sequence(
                scoreOnReefTraj.resetOdometry(),
                scoreOnReefTraj.cmd()
            )
        );

        // Starting at the event marker named "SetTroughToClearAlgea", lower the trough 
        scoreOnReefTraj.atTime("troughStraightOut").onTrue(m_coral.setTroughForClearingAlgeaCommand());
        
        // Starting at the event marker named "ClearAlgea" by raising elevator to L4 
        scoreOnReefTraj.atTime("ClearAlgae").onTrue(m_coral.clearAlgeaCommand());

        // Starting at the event marker named "DropCoarl" score by dropping the trough
        scoreOnReefTraj.atTime("DropCoral").onTrue(m_coral.dumpCoralCommand());

        return routine;
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////////
    /// Auton periodic for non-autofactory control
   
    public void autoPeriotic() {
        /* 
        if (trajectory.isPresent()) {
            // Sample the trajectory at the current time into the autonomous period
           Optional<SwerveSample>   sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

            if (sample.isPresent()) {
               SmartDashboard.putBoolean(   "Sample is Present", sample.isPresent());
               //SmartDashboard.putBoolean(   "Routine Active", goToReefAuto().active().getAsBoolean());

            
                m_robotDrive.followTrajectory(sample.get());
        
            }
        }
        */
        
    }



    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command leftToReefCommand() {
        autonStatus = "Left Wall to Reef";
        SmartDashboard.putString("Attempting", autonStatus);
    
        return Commands.sequence(
            autoFactory.resetOdometry("leftWallToReef"), 
            Commands.deadline(
                autoFactory.trajectoryCmd("leftWallToReef")
                
        ));
        
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command leftOutOfTheWayCommand() {
        autonStatus = "Left Wall out of the way";
        SmartDashboard.putString("Attempting", autonStatus);

        return Commands.sequence(
            autoFactory.resetOdometry("leftWallOutOfTheWay"), 
            Commands.deadline(
                autoFactory.trajectoryCmd("leftWallOutOfTheWay")
                
        ));
        
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command rightToReefCommand() {
        autonStatus = "Right Wall to Reef";
        SmartDashboard.putString("Attempting", autonStatus);

        return Commands.sequence(
            autoFactory.resetOdometry("rightWallToReef"), 
            Commands.deadline(
                autoFactory.trajectoryCmd("rightWallToReef")
                
        ));
        
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////



    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command rightOutOfTheWayCommand() {
        autonStatus = "RightOutOfTheWay";
        SmartDashboard.putString("Attempting", autonStatus);

        return Commands.sequence(
            autoFactory.resetOdometry("rightWallOutOfTheWay"), 
            Commands.deadline(
                autoFactory.trajectoryCmd("rightWallOutOfTheWay")
                
        ));
        
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////




    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command basicBackupCommand() {
        autonStatus = "Basic Backup Command";
        SmartDashboard.putString("Attempting", autonStatus);
        return Commands.sequence(
            
            Commands.run(() -> m_robotDrive.drive(-1.0,0,0,true)).withTimeout(5)
            //Commands.waitSeconds(5),
            //Commands.runOnce(() -> m_robotDrive.drive(0, 0, 0, true)).withTimeout(2)
        );
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 




}
