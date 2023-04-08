// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.util.field.AllianceTransform;
import frc.util.motor.SimpleCurrentLimit;

public class Constants {

  public static class DriveConstants{

    public static final double kTrackwidth = Units.inchesToMeters(19);
    public static final double kDistancePerPulse = (1.0/2048d) * (Units.inchesToMeters(6) * Math.PI) * (1/10.71);
    
    public static final double kMaxSpeed = 6380.0 * (1 / 10.71) * (Units.inchesToMeters(6) * Math.PI) * (1 / 60d);
    public static final double kMaxTurnSpeed = (kMaxSpeed * (1 / (kTrackwidth * Math.PI))) * (2 * Math.PI);

    public static final SupplyCurrentLimitConfiguration kDriveCurrentLimit = SimpleCurrentLimit.getSimpleCurrentLimit(30);

    //Volts
    public static final double kDriveKS = 0.25;
    public static final double kTurnKS = 0.125;

    //PID Controllers
    public static final PIDController kTurnPID = new PIDController(1d / 15, 0, 1d / 300);
    public static final PIDController kAutoDrivePID = new PIDController(2, 0, 0);
    public static final PIDController kTeleDrivePID = new PIDController(0.5, 0, 0);
    public static final PIDController kChargePID = new PIDController(3d/11d, 0, 0);
    public static final PIDController kSnapPID = new PIDController(1d/180, 0, 0);

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
          public static final int kFrontLeft = 1;
          public static final int kBackLeft = 2;
          public static final int kFrontRight = 3;
          public static final int kBackRight = 4;
          public static final int kPigeon = 5;
          public static final int kIntake = 6;
          public static final int kFrontPivot = 7;
          public static final int kBackPivot = 8;

        }

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
    
        public static SupplyCurrentLimitConfiguration kIntakeCurrentLimit = SimpleCurrentLimit.getSimpleCurrentLimit(40);
    
        public static enum State {
          
          GRAB(-0.25),
          RELEASE(0.4),
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

        public static final double kGearing = ((1.0 / 20) * (16.0 / 60));
        public static final double kThroughboreOffset = 0.5149;

        public static final SupplyCurrentLimitConfiguration kPivotCurrentLimit = SimpleCurrentLimit.getSimpleCurrentLimit(25);

        public static enum State {
          SUBSTATION(0),
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