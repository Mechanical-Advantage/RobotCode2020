{
    # Class names of motor controllers used.
    # Options:
    # 'WPI_TalonSRX'
    # 'WPI_TalonFX' (for Falcon 500 motors)
    # 'WPI_VictorSPX'
    # Note: The first motor on each side should always be a Talon SRX/FX, as the
    # VictorSPX does not support encoder connections
    "rightControllerTypes": ["WPI_TalonSRX", "WPI_VictorSPX", "WPI_VictorSPX"],
    "leftControllerTypes": ["WPI_TalonSRX", "WPI_VictorSPX", "WPI_VictorSPX"],
    # Ports for the left-side motors
    "leftMotorPorts": [12, 14, 13],
    # Ports for the right-side motors
    "rightMotorPorts": [3, 2, 1],
    # Inversions for the left-side motors
    "leftMotorsInverted": [True, True, True],
    # Inversions for the right side motors
    "rightMotorsInverted": [True, True, True],
    # Wheel diameter (in units of your choice - will dictate units of analysis)
    "wheelDiameter": 4.633,
    # If your robot has only one encoder, set all right encoder fields to `None`
    # Encoder edges-per-revolution (*NOT* cycles per revolution!)
    # This value should be the edges per revolution *of the wheels*, and so
    # should take into account gearing between the encoder and the wheels
    "encoderEPR": 4096,
    # Whether the left encoder is inverted
    "leftEncoderInverted": True,
    # Whether the right encoder is inverted:
    "rightEncoderInverted": False,
    # Your gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
    "gyroType": "NavX",
    # Whatever you put into the constructor of your gyro
    # Could be:
    # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
    # "I2C.Port.kOnboard" (Onboard I2C port for NavX)
    # "0" (Pigeon CAN ID or AnalogGyro channel),
    # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
    # "leftSlave" (Pigeon on the left slave Talon SRX/FX),
    # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
    "gyroPort": "SPI.Port.kMXP",
}

