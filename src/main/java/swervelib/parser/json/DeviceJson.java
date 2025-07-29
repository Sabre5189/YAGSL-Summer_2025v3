package swervelib.parser.json;

import static swervelib.telemetry.SwerveDriveTelemetry.canIdWarning;
import static swervelib.telemetry.SwerveDriveTelemetry.i2cLockupWarning;
import static swervelib.telemetry.SwerveDriveTelemetry.serialCommsIssueWarning;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import swervelib.encoders.AnalogAbsoluteEncoderSwerve;
import swervelib.encoders.CANCoderSwerve;
import swervelib.encoders.DIODutyCycleEncoderSwerve;
import swervelib.encoders.SparkFlexEncoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.imu.AnalogGyroSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.imu.SwerveIMU;
import swervelib.motors.SparkFlexSwerve;
import swervelib.motors.SparkMaxSwerve;
import swervelib.motors.SwerveMotor;
import swervelib.motors.TalonFXSSwerve;
import swervelib.motors.TalonFXSwerve;
import swervelib.parser.deserializer.ReflectionsManager;
import swervelib.parser.deserializer.ReflectionsManager.VENDOR;

/**
 * Device JSON parsed class. Used to access the JSON data.
 */
public class DeviceJson
{

  /**
   * The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx
   */
  public String type;
  /**
   * The CAN ID or pin ID of the device.
   */
  public int    id;
  /**
   * The CAN bus name which the device resides on if using CAN.
   */
  public String canbus = "";

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the current configuration.
   *
   * @param motor {@link SwerveMotor} of which attached encoders will be created from, only used when the type is
   *              "attached" or "canandencoder".
   * @return {@link SwerveAbsoluteEncoder} given.
   */
  public SwerveAbsoluteEncoder createEncoder(SwerveMotor motor)
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "none":
        return null;
      case "sparkflex_integrated":
      case "sparkflex_attached":
      case "sparkflex_canandmag":
      case "sparkflex_canandcoder":
        return new SparkFlexEncoderSwerve(motor, 360);
      case "ctre_mag":
      case "rev_hex":
      case "throughbore":
      case "am_mag":
      case "dutycycle":
        return new DIODutyCycleEncoderSwerve(id);
      case "thrifty":
      case "ma3":
      case "analog":
        return new AnalogAbsoluteEncoderSwerve(id);
      case "cancoder":
        return new CANCoderSwerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveIMU} from the given configuration.
   *
   * @return {@link SwerveIMU} given.
   */
  public SwerveIMU createIMU()
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "analog":
        return new AnalogGyroSwerve(id);
      case "pigeon2":
        return new Pigeon2Swerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized imu/gyroscope type.");
    }
  }

  /**
   * Create a {@link SwerveMotor} from the given configuration.
   *
   * @param isDriveMotor If the motor being generated is a drive motor.
   * @return {@link SwerveMotor} given.
   */
  public SwerveMotor createMotor(boolean isDriveMotor)
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "talonfxs_neo":
        return new TalonFXSSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getNEO(1));
      case "talonfxs_neo550":
        return new TalonFXSSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getNeo550(1));
      case "talonfxs_vortex":
        return new TalonFXSSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getNeoVortex(1));
      case "talonfxs_minion":
        throw new UnsupportedOperationException("Cannot create minion combination yet"); //new TalonFXSSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getNeoVortex(1));
      case "sparkmax_neo":
      case "neo":
      case "sparkmax":
        return new SparkMaxSwerve(Integer.parseInt(canbus), id, isDriveMotor, DCMotor.getNEO(1));
      case "sparkmax_vortex":
        return new SparkMaxSwerve(Integer.parseInt(canbus), id, isDriveMotor, DCMotor.getNeoVortex(1));
      case "sparkmax_minion":
        throw new UnsupportedOperationException("Cannot create minion combination yet");
      case "sparkmax_neo550":
      case "neo550":
        return new SparkMaxSwerve(Integer.parseInt(canbus), id, isDriveMotor, DCMotor.getNeo550(1));
      case "sparkflex_vortex":
      case "vortex":
      case "sparkflex":
        return new SparkFlexSwerve(Integer.parseInt(canbus), id, isDriveMotor, DCMotor.getNeoVortex(1));
      case "sparkflex_neo":
        return new SparkFlexSwerve(Integer.parseInt(canbus), id, isDriveMotor, DCMotor.getNEO(1));
      case "sparkflex_neo550":
        return new SparkFlexSwerve(Integer.parseInt(canbus), id, isDriveMotor, DCMotor.getNeo550(1));
      case "sparkflex_minion":
        throw new UnsupportedOperationException("Cannot create minion combination yet");
      case "falcon500":
      case "falcon":
        return new TalonFXSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getFalcon500(1));
      case "falcon500foc":
        return new TalonFXSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getFalcon500Foc(1));
      case "krakenx60":
      case "talonfx":
        return new TalonFXSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getKrakenX60(1));
      case "krakenx60foc":
        return new TalonFXSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getKrakenX60Foc(1));
      default:
        throw new RuntimeException(type + " is not a recognized motor type.");
    }

  }
}
