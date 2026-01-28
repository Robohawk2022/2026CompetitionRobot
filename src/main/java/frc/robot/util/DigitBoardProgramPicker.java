package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;
import java.util.function.Supplier;

/**
 * <p>This is an autonomous program selector based on the {@link DigitBoard}.
 * We found that a hardware picker was super-useful for autonomous program
 * selection - it gets around the fact that sometimes the SmartDashboard
 * application will hang or act funny.</p>
 *
 * <p>This class assumes each program name has a short codename (e.g. "PRG1"),
 * and a longer name corresponding to the PathPlanner file (e.g. "Exit Left").
 * The short code is displayed to the operator, and the longer name is what
 * gets returned from {@link #get()}.</p>
 *
 * <p>If you're running in simulation, this will publish a picker to the
 * dashboard; on a real robot it will use the {@link DigitBoard}.</p>
 */
public class DigitBoardProgramPicker implements Supplier<String> {

    final Map<String,String> options;
    final String [] shortNames;
    final DigitBoard board;
    final SendableChooser<String> chooser;
    int which;

    /**
     * Creates a {@link DigitBoardProgramPicker}.
     *
     * @param name the name in SmartDashboard (if running in simulation)
     * @param options a code/value mapping of options - the codes will be shown
     *                on the DigitBoard, so they should be short (4 characters
     *                max); the values are the names of the corresponding
     *                autonomous programs
     */
    public DigitBoardProgramPicker(String name, Map<String,String> options) {

        this.options = options;
        this.shortNames = options.keySet().toArray(new String[0]);
        this.which = 0;

        // if we're in simulation, we'll display the options on the dashboard
        // (the first one will be the default option)
        if (RobotBase.isSimulation()) {

            this.board = null;
            this.chooser = new SendableChooser<>();

            boolean first = true;
            for (String code : options.keySet()) {
                if (first) {
                    chooser.setDefaultOption(code, options.get(code));
                    first = false;
                } else {
                    chooser.addOption(code, options.get(code));
                }
            }

            SmartDashboard.putData(name, chooser);

        }

        // if we're using a real board, we will initialize it (the display
        // gets updated when someone calls <code>get</code>
        else {
            this.board = new DigitBoard();
            this.chooser = null;
        }
    }

    /**
     * @return the currently selected program name (you should call this
     * during a <code>periodic</code> method because it also updates the
     * display on the digit board if there is one)
     */
    @Override
    public String get() {
        return board == null ? getFromDashboard() : getFromDigitBoard();
    }

    /*
     * Gets the currently-selected program on the dashboard
     */
    private String getFromDashboard() {
        return chooser.getSelected();
    }

    /*
     * Updates the current selection based on button pushes, and returns the
     * currently selected program name
     */
    private String getFromDigitBoard() {

        // if we have no options available, there's nothing to do
        if (options.isEmpty()) {
            board.display("????");
            return null;
        }

        // this logic will increase or decrease the "which" counter by 1
        // when someone presses and releases the A or B button, which lets
        // them scroll through the program options

        if (board.wasButtonBReleased()) {
            // if they want to go "forwards" past the end of the list, the
            // mod operator will "wrap around" back to the front (item 0)
            which = (which + 1) % options.size();
        }

        if (board.wasButtonAReleased()) {
            which = which - 1;

            // if they want to go "backwards" past the beginning of the
            // the list, we "wrap around" to the end (item N-1)
            if (which < 0) {
                which = options.size() - 1;
            }
        }

        // display the short version of the name on the board
        String shortName = shortNames[which];
        board.display(shortName);

        // look up the longer version and return it
        return options.get(shortName);
    }
}
