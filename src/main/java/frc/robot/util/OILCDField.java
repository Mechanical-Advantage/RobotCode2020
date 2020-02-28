/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Represents a field for the LCD on the operator panel
 */
public class OILCDField {

    private final NetworkTable table;
    private final int x;
    private final int y;
    private final int length;
    private String value;

    /**
     * Initializes field parameters and creates table
     * 
     * @param key          Arbitrary field identifier (not visible on LCD)
     * @param x            X location of this field (0 is left)
     * @param y            location a/k/a line of this field (0 is top)
     * @param lengthLength Length of this field (will be truncated)
     * @param initialValue Initial text to display
     */
    public OILCDField(String key, int x, int y, int length, String initialValue) {
        table = NetworkTableInstance.getDefault().getTable("OperatorInterface/LCD/" + key);
        this.x = x;
        this.y = y;
        this.length = length;
        value = initialValue;
        updateTable();
    }

    /**
     * Updates the field text
     * 
     * @param value New text to display
     */
    public void setValue(String value) {
        this.value = value;
        updateTable();
    }

    private void updateTable() {
        table.getEntry("X").setNumber(x);
        table.getEntry("Y").setNumber(y);
        table.getEntry("Length").setNumber(length);
        table.getEntry("String").setString(value);
    }

    /**
     * Clears all fields in Network Tables. Run before initalizaing fields
     */
    public static void clearFields() {
        NetworkTableInstance.getDefault().getTable("OperatorInterface").delete("LCD");
    }
}
