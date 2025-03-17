// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.Constants.ClimberConstants; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberSubsystem extends SubsystemBase {
 
  /** Creates a new Climber. */
  private final SparkMax m_leftWristSpark; 
  private final SparkMax m_rightWristSpark;  
  private final SparkMax m_climberSpark;

  private final RelativeEncoder m_leftWristEncoder;
  private final RelativeEncoder m_rightWristEncoder;
  private final RelativeEncoder m_climberEncoder;

  private final SparkClosedLoopController m_leftWristClosedLoopController;
  private final SparkClosedLoopController m_rightWristClosedLoopController;
  private final SparkClosedLoopController m_climberClosedLoopController;

  public ClimberSubsystem() {

    m_leftWristSpark = new SparkMax(ClimberConstants.kleftWristCANId, MotorType.kBrushless);
    m_rightWristSpark = new SparkMax(ClimberConstants.krightWristCANId, MotorType.kBrushless);
    m_climberSpark = new SparkMax(ClimberConstants.kclimberCANId, MotorType.kBrushless);
    
    m_climberEncoder = m_climberSpark.getEncoder();
    m_leftWristEncoder = m_leftWristSpark.getEncoder();
    m_rightWristEncoder = m_rightWristSpark.getEncoder();

    m_leftWristClosedLoopController = m_leftWristSpark.getClosedLoopController();
    m_rightWristClosedLoopController = m_rightWristSpark.getClosedLoopController();
    m_climberClosedLoopController = m_climberSpark.getClosedLoopController();

    m_leftWristSpark.configure(Configs.Climber.leftwristConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    m_rightWristSpark.configure(Configs.Climber.rightwristConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    m_climberSpark.configure(Configs.Climber.climberConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }

  


  public void setLeftWrist (int leftwristPosition) {
    m_leftWristClosedLoopController.setReference(leftwristPosition, ControlType.kPosition);
  }
  
  public void setRightWrist (int rightwristPosition) {
    m_rightWristClosedLoopController.setReference(rightwristPosition, ControlType.kPosition);
  }
 

  public void setWrists (int left, int right) {
    setLeftWrist(left);
    setRightWrist(right);
  }

  public void setClimber (int climberPosition) {
    m_climberClosedLoopController.setReference(climberPosition, ControlType.kPosition);
  }



}
