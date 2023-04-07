// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.field.AllianceTransform;
import frc.util.motor.SimpleCurrentLimit;

public class Constants {

  public static class DriveConstants{

    public static double kTrackwidth = Units.inchesToMeters(19);
    public static double kDistancePerPulse = (1.0/2048d) * (Units.inchesToMeters(6) * Math.PI) * (1/10.71);
    
    public static double kMaxSpeed = 6380.0 * (1 / 10.71) * (Units.inchesToMeters(6) * Math.PI) * (1 / 60d);
    public static double kMaxTurnSpeed = (kMaxSpeed * (1 / (kTrackwidth * Math.PI))) * (2 * Math.PI);

    public static SupplyCurrentLimitConfiguration kDriveCurrentLimit = SimpleCurrentLimit.getSimpleCurrentLimit(30);

    //Volts
    public static double kDriveKS = 0.25;
    public static double kTurnKS = 0.125;

    //PID Controllers
    public static PIDController kTurnPID = new PIDController(1d / 15, 0, 1d / 300);
    public static PIDController kAutoDrivePID = new PIDController(2, 0, 0);
    public static PIDController kTeleDrivePID = new PIDController(0.5, 0, 0);
    public static PIDController kChargePID = new PIDController(3d/11d, 0, 0);

    public static enum DirState {
     
      FORWARD(1),
      REVERSE(-1);

      public final double direction;

      /**
       * @param direction Motor Percentage
       */

      DirState(double direction) {
        this.direction = direction;
      }
    }

    public static enum SpeedState {
     
      TURBO(1, 0.8),
      NORMAL(0.8, 0.2),
      SLOW(0.6, 0.2);
  
      public final double xMod;
      public final double rotMod;
  
      /**
       * @param modifier Turbo or Slow
       */
  
      SpeedState(double xMod, double rotMod) {
        this.xMod = xMod;
        this.rotMod = rotMod;
      }
    }

   }



    public static class RobotConstants{
        public static class CAN {
          public static int kFrontLeft = 1;
          public static int kBackLeft = 2;
          public static int kFrontRight = 3;
          public static int kBackRight = 4;
          public static int kPigeon = 5;
          public static int kIntake = 6;
          public static int kFrontPivot = 7;
          public static int kBackPivot = 8;

        }

        public static BooleanSupplier isBlue = () -> DriverStation.getAlliance() == Alliance.Blue;

        public enum StartingPose {
     
          BLUE_SUB(new Pose2d(new Translation2d(2.2, 4.45), Rotation2d.fromDegrees(-90))),
          BLUE_MID(new Pose2d(new Translation2d(2.2, 2.75), Rotation2d.fromDegrees(-90))),
          BLUE_BUMP(new Pose2d(new Translation2d(2.2, 1), Rotation2d.fromDegrees(-90))),

          RED_SUB(AllianceTransform.flipAlliance(BLUE_SUB.pose)),
          RED_MID(AllianceTransform.flipAlliance(BLUE_MID.pose)),
          RED_BUMP(AllianceTransform.flipAlliance(BLUE_BUMP.pose));

          
          public final Pose2d pose;
    
          /**
           * @param direction Motor Percentage
           */
    
          StartingPose(Pose2d pose) {
            this.pose = pose;
          }
        }

    }

    public static class IntakeConstants {
    
        public static final double kInSpeed = 0.25;
        public static final double kOutSpeed = 0.40;

        public static SupplyCurrentLimitConfiguration kIntakeCurrentLimit = SimpleCurrentLimit.getSimpleCurrentLimit(30);
    
        public static enum State {
          
          GRAB(-kInSpeed),
          RELEASE(kOutSpeed),
          L1RELEASE(0.1),
          L2RELEASE(0.15),
          L3RELEASE(0.25),
          STOP(0);
    
          public final double speed;
    
          /**
           * @param speed Motor Percentage
           */
          State(double speed) {
            this.speed = speed;
          }
    
        }
    
      }

      public static class PivotConstants {

        public static double kGearing = ((1.0 / 20) * (16.0 / 60));
        public static double kThroughboreOffset = 0.5149;

        public static SupplyCurrentLimitConfiguration kPivotCurrentLimit = SimpleCurrentLimit.getSimpleCurrentLimit(25);

        public static enum State {
          SUBSTATION(5),
          L1(45),
          L2(-20),
          L3(-30),
          STOP(0),
          STARTING(-90),
          FLOOR(110),
          CARRY(-90);
    
          public final double angle;
    
          /**
           * @param angle Pivot Angle
           */
          State(double angle) {
            this.angle = angle;
          }
    
        }
    
      }
}