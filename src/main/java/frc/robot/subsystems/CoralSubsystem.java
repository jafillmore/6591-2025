// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  private final SparkMax m_troughSpark;
  private final SparkMax m_elevatorSpark; 
    
  private final AbsoluteEncoder m_troughEncoder;
  private final RelativeEncoder m_elvatorEncoder;

  private final SparkClosedLoopController m_troughClosedLoopController;
  private final SparkClosedLoopController m_elevatorClosedLoopController;

  //m_troughClosedLoopController = m_troughSpark.getClosedLoopController();

  public CoralSubsystem() {

    m_troughSpark = new SparkMax(CoralConstants.ktroughCANId, MotorType.kBrushless);
    m_elevatorSpark = new SparkMax(CoralConstants.kelevatorCANId, MotorType.kBrushless);

    m_troughEncoder = m_troughSpark.getAbsoluteEncoder();
    m_elvatorEncoder = m_elevatorSpark.getEncoder();

    m_troughClosedLoopController = m_troughSpark.getClosedLoopController();
    m_elevatorClosedLoopController = m_troughSpark.getClosedLoopController();


    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_troughSpark.configure(Configs.Coral.troughConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_elevatorSpark.configure(Configs.Coral.elevatorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void ejectCoral (double CoralPower) {
    m_troughSpark.set(CoralPower);

  };



  



}
