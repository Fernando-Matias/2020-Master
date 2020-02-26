package frc.robot.utility.geometry;

import frc.robot.utility.util.CSVWritable;
import frc.robot.utility.util.Interpolable;   

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
