// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;


/** Container for auto command factories. */
public final class Autos {
  
  //////////////////////////////////////
  //////////  RED Auto 1 ///////////////
  //////////////////////////////////////

  public static Command redAuto1(DriveSubsystem drive) {

    
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // Trajectory straight out to grab a note...
    Trajectory red1Trajectory1 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0), new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-2.5, 0, new Rotation2d(0)),
        config);
    Trajectory red1Trajectory2 = TrajectoryGenerator.generateTrajectory(
        // Start at 2.5m out facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0), new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-2.5, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand red1Trajectory1ControllerCommand = new SwerveControllerCommand(
        red1Trajectory1,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    SwerveControllerCommand red1Trajectory2ControllerCommand = new SwerveControllerCommand(
        red1Trajectory2,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(red1Trajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    return red1Trajectory1ControllerCommand.andThen(() -> drive.drive(0, 0, 0, false));

    }

  //////////////////////////////////////
  //////////  RED Auto 2 ///////////////
  //////////////////////////////////////

  public static Command redAuto2(DriveSubsystem drive) {
  // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory red2Trajectory1 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0), new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-2.5, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand red2Trajectory1ControllerCommand = new SwerveControllerCommand(
        red2Trajectory1,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(red2Trajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    return red2Trajectory1ControllerCommand.andThen(() -> drive.drive(0, 0, 0, false));
  }

  //////////////////////////////////////
  //////////  RED Auto 3 ///////////////
  //////////////////////////////////////

  public static Command redAuto3(DriveSubsystem drive) {
  // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory red3Trajectory1 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0), new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-2.5, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand red3Trajectory1ControllerCommand = new SwerveControllerCommand(
        red3Trajectory1,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(red3Trajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    return red3Trajectory1ControllerCommand.andThen(() -> drive.drive(0, 0, 0, false));
  }


  //////////////////////////////////////
  //////////  BLUE Auto 1 ///////////////
  //////////////////////////////////////

  public static Command blueAuto1(DriveSubsystem drive) {

    
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // Trajectory straight out to grab a note...
    Trajectory blue1Trajectory1 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0), new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-2.5, 0, new Rotation2d(0)),
        config);

    Trajectory blue1Trajectory2 = TrajectoryGenerator.generateTrajectory(
        // Start at 2.5m out facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0), new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-2.5, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand blue1Trajectory1ControllerCommand = new SwerveControllerCommand(
        blue1Trajectory1,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    SwerveControllerCommand blue1Trajectory2ControllerCommand = new SwerveControllerCommand(
        blue1Trajectory2,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(blue1Trajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    return blue1Trajectory1ControllerCommand.andThen(() -> drive.drive(0, 0, 0, false));
    
    }

 
  //////////////////////////////////////
  //////////  BLUE Auto 2 ///////////////
  //////////////////////////////////////

  public static Command blueAuto2(DriveSubsystem drive) {
  // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory blue2Trajectory1 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1.0, 0), new Translation2d(-2.0, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-2.5, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand blue2Trajectory1ControllerCommand = new SwerveControllerCommand(
        blue2Trajectory1,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(blue2Trajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    return blue2Trajectory1ControllerCommand.andThen(() -> drive.drive(0, 0, 0, false));
  }

  //////////////////////////////////////
  //////////  BLUE Auto 3 ///////////////
  //////////////////////////////////////

  public static Command blueAuto3(DriveSubsystem drive) {
  // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory blue3Trajectory1 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0), new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-2.5, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand blue3Trajectory1ControllerCommand = new SwerveControllerCommand(
        blue3Trajectory1,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(blue3Trajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    return blue3Trajectory1ControllerCommand.andThen(() -> drive.drive(0, 0, 0, false));
    }


    //String trajectoryJSON = "\"C:\\Users\\Joel\\Documents\\FRC\\2024\\Code\\PathWeaver\\output\\output\\Unnamed.wpilib.json\"";
    //Trajectory trajectory = new Trajectory();


    public void robotInit() {
    /*   
        try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    */
    }




    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
