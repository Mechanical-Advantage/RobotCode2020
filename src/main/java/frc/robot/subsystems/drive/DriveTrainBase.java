package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveTrainBase extends SubsystemBase {

  protected double kPLow;
  protected double kILow;
  protected double kDLow;
  protected double kFLow;
  protected int kIZoneLow;
  protected double kPHigh;
  protected double kIHigh;
  protected double kDHigh;
  protected double kFHigh;
  protected int kIZoneHigh;

  protected double wheelDiameter; // inches
  protected boolean dualGear = false;
  protected boolean hasPTO = false;
  protected double PTORightSpeedAdjust; // Multiplier applied to right side setpoint when driving the PTO
  protected double PTOLeftSpeedAdjust;
  protected double maxVelocityLow;
  protected double maxVelocityHigh;
  protected double minVelocityLow;
  protected double minVelocityHigh;
  protected int leftGearPCM;
  protected int leftGearSolenoid1;
  protected int leftGearSolenoid2;
  protected int rightGearPCM;
  protected int rightGearSolenoid1;
  protected int rightGearSolenoid2;
  protected int ptoPCM;
  protected int ptoSolenoid1;
  protected int ptoSolenoid2;

  private DoubleSolenoid pto;
  private DoubleSolenoid leftGearSolenoid;
  private DoubleSolenoid rightGearSolenoid;
  private BooleanSupplier driveDisableSwitchAccess;
  private BooleanSupplier openLoopSwitchAccess;
  private BooleanSupplier shiftLockSwitchAccess;

  private DriveControlMode currentControlMode = DriveControlMode.STANDARD_DRIVE; // enum defined at end of file
  private DriveGear currentGear = null;

  /*
   * Profile Slots:
   * 
   * 0 - Low gear
   * 
   * 1 - High gear
   * 
   * These are used regardless of the subclass type and are managed entirely by
   * the base class (the subclass shouldn't need to care about this mapping)
   */

  public DriveTrainBase(BooleanSupplier driveDisableSwitchAccess, BooleanSupplier openLoopSwitchAccess,
      BooleanSupplier shiftLockSwitchAccess) {
    this.driveDisableSwitchAccess = driveDisableSwitchAccess;
    this.openLoopSwitchAccess = openLoopSwitchAccess;
    this.shiftLockSwitchAccess = shiftLockSwitchAccess;
  }

  /**
   * Creates solenoids and configures profile slots. Required before using
   * methods.
   */
  protected void initialize() {
    if (dualGear) {
      leftGearSolenoid = new DoubleSolenoid(leftGearPCM, leftGearSolenoid1, leftGearSolenoid2);
      rightGearSolenoid = new DoubleSolenoid(rightGearPCM, rightGearSolenoid1, rightGearSolenoid2);
      setPID(1, kPHigh, kIHigh, kDHigh, kFHigh, kIZoneHigh);
      switchGear(DriveGear.HIGH);
    }
    setPID(0, kPLow, kILow, kDLow, kFLow, kIZoneLow);
    if (hasPTO) {
      pto = new DoubleSolenoid(ptoPCM, ptoSolenoid1, ptoSolenoid2);
      disablePTO();
    }
  }

  /**
   * Gets the maximum velocity for the current robot in the current gear.
   * 
   * @return The maximum velocity in inches per second
   */
  public double getMaxVelocity() {
    if (!dualGear || currentGear == DriveGear.LOW) {
      return maxVelocityLow;
    } else {
      return maxVelocityHigh;
    }
  }

  /**
   * Drives the robot with speed specified as inches per second
   * 
   * @param left  Left inches per second
   * @param right Right inches per second
   */
  public void driveInchesPerSec(int left, int right) {
    driveInchesPerSec((double) right, (double) left);
  }

  /**
   * Drives the robot with speed specified as inches per second
   * 
   * @param left  Left inches per second
   * @param right Right inches per second
   */
  public void driveInchesPerSec(double left, double right) {
    if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
      if (driveDisableSwitchAccess.getAsBoolean()) {
        left = 0;
        right = 0;
      }

      if (openLoopSwitchAccess.getAsBoolean()) {
        driveOpenLoopLowLevel(calcActualVelocity(left, false) / getMaxVelocity(),
            calcActualVelocity(right, false) / getMaxVelocity());
      } else {
        driveClosedLoopLowLevel((calcActualVelocity(left, false) / (wheelDiameter * Math.PI)),
            (calcActualVelocity(right, false) / (wheelDiameter * Math.PI)));
      }
    }
  }

  /**
   * Increases inputs that are less than minVelocity to minVelocity.
   * 
   * @param input        Input value, either inches per second or percentage
   * @param isPercentage Whether the input is a percentage
   * @return Calculated velocity
   */
  private double calcActualVelocity(double input, boolean isPercentage) {
    double minVelocity;
    if (!dualGear || currentGear == DriveGear.LOW) {
      minVelocity = minVelocityLow;
    } else {
      minVelocity = minVelocityHigh;
    }
    double minNonZero = 0.1;
    if (isPercentage) {
      minVelocity /= getMaxVelocity();
      minNonZero = 0.001;
    }
    if (input > minNonZero * -1 && input < minNonZero) {
      return 0;
    } else if (input >= minNonZero && input < minVelocity) {
      return minVelocity;
    } else if (input <= minNonZero * -1 && input > minVelocity * -1) {
      return minVelocity * -1;
    } else {
      return input;
    }
  }

  /**
   * Drives the robot
   * 
   * @param left  Left percent speed
   * @param right Right percent speed
   */
  public void drive(double left, double right) {
    drive(left, right, false);
  }

  /**
   * Drives the robot.
   * 
   * @param left             Left percent speed
   * @param right            Right percent speed
   * @param alwaysHighMaxVel Whether to always use the max velocity of high gear
   *                         or of current gear
   */
  public void drive(double left, double right, boolean alwaysHighMaxVel) {
    if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
      if (driveDisableSwitchAccess.getAsBoolean()) {
        left = 0;
        right = 0;
      }

      double maxVelocity = getMaxVelocity();
      if (dualGear && alwaysHighMaxVel && currentGear == DriveGear.LOW) {
        maxVelocity = maxVelocityHigh;
      }
      left = calcActualVelocity(left, true);
      right = calcActualVelocity(right, true);
      // If in closed loop, convert max velocity from inches per second to rotations
      // per second to match input unit of driveClosedLoopLowLevel
      if (!openLoopSwitchAccess.getAsBoolean()) {
        maxVelocity /= wheelDiameter * Math.PI;
      }
      left *= maxVelocity;
      right *= maxVelocity;

      if (openLoopSwitchAccess.getAsBoolean()) {
        driveOpenLoopLowLevel(left / getMaxVelocity(), right / getMaxVelocity());
      } else {
        driveClosedLoopLowLevel(left, right);
      }
    }
  }

  /**
   * Stops the drive. Note that this function may drive the motors opposite the
   * current direction of motion to stop more quickly. Use neutralOutput() to
   * completely stop output.
   */
  public void stop() {
    drive(0, 0);
  }

  /**
   * Stops all power to the drive motors. This is different than stop() because it
   * will never use motor power to stop more quickly.
   */
  public abstract void neutralOutput();

  /**
   * Internal method to directly send an open loop setpoint to the motor
   * controllers.
   * 
   * @param left  The left percent output (-1 to 1)
   * @param right The right percent output (-1 to 1)
   */
  protected abstract void driveOpenLoopLowLevel(double left, double right);

  /**
   * Internal method to directly send a closed loop setpoint to the motor
   * controllers.
   * 
   * @param left  The left velocity (rotations per second)
   * @param right The right velocity (rotations per second)
   */
  protected abstract void driveClosedLoopLowLevel(double left, double right);

  /**
   * Enables or disables brake mode (shorting motor terminals to create braking
   * force when the motor controller outputs 0V)
   * 
   * @param enable Whether to enable brake mode
   */
  public abstract void enableBrakeMode(boolean enable);

  /**
   * Resets the encoder position returned by getRotations/getDistance functions
   */
  public abstract void resetPosition();

  /**
   * Gets the number of rotations of the left side of the drive since the last
   * call to resetPosition.
   * 
   * @return Number of rotations
   */
  public abstract double getRotationsLeft();

  /**
   * Gets the number of rotations of the right side of the drive since the last
   * call to resetPosition.
   * 
   * @return Number of rotations
   */
  public abstract double getRotationsRight();

  /**
   * Gets the distance travelled of the right side of the drive since the last
   * call to resetPosition.
   * 
   * @return Distance in inches
   */
  public double getDistanceRight() {
    return wheelDiameter * Math.PI * getRotationsRight();
  }

  /**
   * Gets the distance travelled of the left side of the drive since the last call
   * to resetPosition.
   * 
   * @return Distance in inches
   */
  public double getDistanceLeft() {
    return wheelDiameter * Math.PI * getRotationsLeft();
  }

  /**
   * Get the current velocity for the right side of the robot
   * 
   * @return current velocity in inches per second
   */
  public double getVelocityRight() {
    return wheelDiameter * Math.PI * getRPSRight();
  }

  /**
   * Get the current velocity for the right side of the robot
   * 
   * @return current velocity in rotations per second
   */
  protected abstract double getRPSRight();

  /**
   * Get the current velocity for the left side of the robot
   * 
   * @return current velocity in inches per second
   */
  public double getVelocityLeft() {
    return wheelDiameter * Math.PI * getRPSLeft();
  }

  /**
   * Get the current velocity for the left side of the robot
   * 
   * @return current velocity in rotations per second
   */
  protected abstract double getRPSLeft();

  /**
   * Gets the average current for one motor.
   * 
   * @return The current of one motor in amps
   */
  public abstract double getCurrent();

  /**
   * Sets the PID parameters for the current control mode, useful for tuning.
   * Calling this effects everything using the subsystem, use with care.
   * 
   * @param p     P
   * @param i     I
   * @param d     D
   * @param f     F
   * @param iZone Integral zone
   */
  public void setPID(double p, double i, double d, double f, int iZone) {
    int slot;
    if (currentControlMode == DriveControlMode.STANDARD_DRIVE && (currentGear == DriveGear.LOW || !dualGear)) {
      slot = 0;
    } else if (currentControlMode == DriveControlMode.STANDARD_DRIVE && currentGear == DriveGear.HIGH) {
      slot = 1;
    } else {
      slot = -1;
    }
    if (slot >= 0) {
      setPID(slot, p, i, d, f, iZone);
    }
  }

  /**
   * Sets the PID parameters for the given slot on the motor controller
   * 
   * @param slotIdx Which slot to write to
   * @param p       P
   * @param i       I
   * @param d       D
   * @param f       F
   * @param iZone   Integral zone
   */
  protected abstract void setPID(int slotIdx, double p, double i, double d, double f, int iZone);

  public double getP() {
    if (currentGear == DriveGear.HIGH) {
      return kPHigh;
    } else {
      return kPLow;
    }
  }

  public double getI() {
    if (currentGear == DriveGear.HIGH) {
      return kIHigh;
    } else {
      return kILow;
    }
  }

  public double getD() {
    if (currentGear == DriveGear.HIGH) {
      return kDHigh;
    } else {
      return kDLow;
    }
  }

  public double getF() {
    if (currentGear == DriveGear.HIGH) {
      return kFHigh;
    } else {
      return kFLow;
    }
  }

  public int getIZone() {
    if (currentGear == DriveGear.HIGH) {
      return kIZoneHigh;
    } else {
      return kIZoneLow;
    }
  }

  /**
   * Changes the time between status frames. This is how often the follower
   * controllers will update their output.
   * 
   * @param ms How many milliseconds to wait between frames
   */
  public abstract void changeStatusRate(int ms);

  /**
   * Resets the status rate to the default value.
   */
  public abstract void resetStatusRate();

  /**
   * Changes the time between sensor frames. This is how often the
   * getRotations/getDistance/getVelocity functions will update.
   * 
   * @param ms How many milliseconds to wait between frames
   */
  public abstract void changeSensorRate(int ms);

  /**
   * Resets the sensor rate to the default value.
   */
  public abstract void resetSensorRate();

  /**
   * Changes the time between control frames. This is how often a setpoint is sent
   * to the controller.
   * 
   * @param ms How many milliseconds to wait between frames
   */
  public abstract void changeControlRate(int ms);

  /**
   * Resets the control rate to the default value.
   */
  public abstract void resetControlRate();

  /**
   * Changes which PID slot is selected on the motor controllers.
   * 
   * @param slotIdx Which slot (0-3)
   */
  protected abstract void setProfileSlot(int slotIdx);

  public void switchGear(DriveGear gear) {
    if (dualGear && !shiftLockSwitchAccess.getAsBoolean()) {
      switch (gear) {
      case HIGH:
        leftGearSolenoid.set(Value.kForward);
        rightGearSolenoid.set(Value.kForward);
        setProfileSlot(1);
        currentGear = DriveGear.HIGH;
        SmartDashboard.putBoolean("High Gear", true);
        break;
      case LOW:
        leftGearSolenoid.set(Value.kReverse);
        rightGearSolenoid.set(Value.kReverse);
        setProfileSlot(0);
        currentGear = DriveGear.LOW;
        SmartDashboard.putBoolean("High Gear", false);
        break;
      case UNSUPPORTED:
      default:
        break;
      }
    }
  }

  public DriveGear getCurrentGear() {
    if (dualGear) {
      return currentGear;
    } else {
      return DriveGear.UNSUPPORTED;
    }
  }

  /**
   * Gets whether the current robot has a two speed gearbox.
   * 
   * @return Whether gear switching is supported
   */
  public boolean isDualGear() {
    return dualGear;
  }

  /**
   * Engages the PTO.
   */
  public void enablePTO() {
    if (hasPTO) {
      currentControlMode = DriveControlMode.PTO;
      neutralOutput();
      pto.set(Value.kForward);
    }
  }

  /**
   * Disengages the PTO.
   */
  public void disablePTO() {
    if (currentControlMode == DriveControlMode.PTO) {
      neutralOutput();
      pto.set(Value.kReverse);
      currentControlMode = DriveControlMode.STANDARD_DRIVE;
    }
  }

  /**
   * Drives the PTO.
   * 
   * @param speed The percent speed (-1 to 1)
   */
  public void runPTO(double speed) {
    if (!driveDisableSwitchAccess.getAsBoolean() && currentControlMode == DriveControlMode.PTO) {
      driveOpenLoopLowLevel(speed * PTOLeftSpeedAdjust, speed * PTORightSpeedAdjust);
    } else if (driveDisableSwitchAccess.getAsBoolean()) {
      neutralOutput();
    }
  }

  private enum DriveControlMode {
    STANDARD_DRIVE, PTO
  }

  public enum DriveGear {
    HIGH, LOW, UNSUPPORTED;

    public DriveGear invert() {
      switch (this) {
      case HIGH:
        return LOW;
      case LOW:
        return HIGH;
      default:
        return UNSUPPORTED;
      }
    }
  }
}