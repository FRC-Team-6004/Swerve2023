package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public final class Constants {

    public static final class ModuleConstants {

        public static final double kEncoderCPR = 2048;

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.14; // L1
        public static final double kTurningMotorGearRatio = 1 / (150 / 7);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = 0.58; // meters
        // Distance between right and left wheels
        public static final double kWheelBase = 0.58; // meters
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 7;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 10;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackLeftTurningMotorPort = 8;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 11;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 3;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 9;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 6;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -(-1.524768); // offset in radians
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -(-0.510815); // offset in radians
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -(-0.542895); // offset in radians
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -(-2.258015); // offset in radians

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.259214240103287;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 1; // change
                                                                                                               // denomenator
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * .75; // change denomenator
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class MechanismConstants {
        public static final int kTelescopePort = 20;
        public static final int kPivotPort = 21;
        public static final int kPivotFollowPort = 22;

        public static final int kTelescopeEncoderCPR = 2048; // count per revolution
        public static final int kPivotEncoderCPR = 42; // counts per revolution

        public static final double kReductionTelescope = 35;
        public static final double kRotationsToFullExtentTelescope = 8;
        public static final double kReductionPivot = 100 * (56 / 18);

        public static final double kPTelescope = 0.5;
        public static final double kITelescope = 0;
        public static final double kDTelescope = 0;
        public static final double kFTelescope = 0;
        public static final double kMinTelescope = 1;
        public static final double kMaxTelescope = 1;
        public static final int kTimeoutMsTelescope = 30;
        public static final int kPIDLoopIdxTelescope = 0;
        public static final int kSlotIdxTelescope = 0;

        public static final double kPPivot = 4;
        public static final double kIPivot = 0;
        public static final double kDPivot = 0;
        public static final double kMinPivot = -.35;
        public static final double kMaxPivot = .35;

    }

    public static final class AutoConstants {

        public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 0.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.75;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.75; // multiplier for controller PID control
        public static final double kPYController = 0.75; // multiplier for controller PID control
        public static final double kPThetaController = 1.75; // multiplier for controller PID control
        public static final double kPBalanceController = 2; // multiplier for controller PID control
        public static final double kRangeBalance = 0.55; // max speed for balancing
        public static final double kRangeBalanceClose = 0.35; // max speed for balancing

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        // public static final int kDriverRobotOrientedButtonIdx = 6;
        public static final int kDriverBrakeButton = 6;

        public static final int kAlignWithTargetButton = 5;
        public static final int kResetDirectionButton = 4;

        public static final int kRotate0Button = 2;
        public static final int kRotate180Button = 3;
        public static final int kExtendFullButton = 4;
        public static final int kRetractButton = 1;
        public static final int kToggleGrabButton = 10;
        public static final int kReverseGrabButton = 6;
        public static final int kForwardGrabButton = 5;
        public static final int kManuelButton = 7;

        public static final int kSlowModeAxis = 2;
        public static final int kAutoEngageButton = 1;

        public static final double kDeadband = 0.05;
    }

    public enum ModulePosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }
}