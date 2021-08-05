/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;
import java.util.Map;

import frc.robot.Constants.RobotType;
import frc.robot.util.Alert.AlertType;

/**
 * Identifies a roboRIO by its MAC Address
 */
public class RobotIdentification {
    private static final String networkInterface = "eth0";
    // Java bytes are signed so standard hex notation won't work
    private static final Map<MACAddress, RobotType> robotMACs = Map.of(
            new MACAddress(new byte[] { 0, -128, 47, 37, 122, -105 }), RobotType.ROBOT_2020,
            new MACAddress(new byte[] { 0, -128, 47, 23, -47, 95 }), RobotType.ORIGINAL_ROBOT_2018,
            new MACAddress(new byte[] { 0, -128, 47, 36, 78, 94 }), RobotType.NOTBOT,
            new MACAddress(new byte[] { 0, -128, 47, 35, -30, 92 }), RobotType.ROBOT_2020_DRIVE);

    public static RobotType identifyRobot() {
        try {
            MACAddress macAddress = new MACAddress(NetworkInterface.getByName(networkInterface).getHardwareAddress());
            RobotType robot = robotMACs.get(macAddress);
            if (robot == null) {
                new Alert("Could not identify MAC '" + Arrays.toString(macAddress.getAddress())
                        + "', using default robot instead.", AlertType.WARNING).set(true);
            } else {
                System.out.println("Identified MAC address as " + robot);
            }
            return robot;
        } catch (SocketException | NullPointerException err) {
            new Alert("Failed to read MAC, using default robot instead.", AlertType.WARNING).set(true);
            return null;
        }
    }

    // This class can be used in maps, unlike a raw byte[]
    /**
     * A class that stores a MAC address that can be used in data structures. It
     * implements equals and hashCode.
     */
    private static final class MACAddress {
        private final byte[] address;

        public MACAddress(byte[] address) {
            if (address == null) {
                throw new NullPointerException();
            }
            this.address = address;
        }

        public byte[] getAddress() {
            return address;
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof MACAddress)) {
                return false;
            }
            return Arrays.equals(address, ((MACAddress) other).address);
        }

        @Override
        public int hashCode() {
            return Arrays.hashCode(address);
        }
    }
}
