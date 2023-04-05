// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.IntakeConstants.State;
import frc.robot.Constants.RobotConstants.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX mMotor;
  private State mCurrentState;
  public Intake() {

    mMotor = new WPI_TalonSRX(CAN.kIntake);
    mMotor.configVoltageCompSaturation(RobotConstants.maxVoltage);
    mMotor.enableVoltageCompensation(true);
    mMotor.setNeutralMode(NeutralMode.Coast);
    mMotor.setInverted(true);
    mMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 15, 0));

    mCurrentState = State.STARTING;
  }

  private void runIntake(){
    mMotor.set(mCurrentState.speed);
  }

  public Command changeState(State state){
    return new InstantCommand(() -> mCurrentState = state);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("IntakeSet", mCurrentState.speed);

    runIntake();
  }
}
