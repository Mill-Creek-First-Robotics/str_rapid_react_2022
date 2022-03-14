package frc.robot.subsystems;

/** A subsystem for waiting a certain amount of time.
 * @author Austin Mann
 * @version 1.0
 * @since 1.0
*/
public class timer {

    /** Waits for the amount of time in milliseconds specified before continuing.
     * @param milliseconds  the amount of time to wait in milliseconds.
    */
    public static void sleep(int milliseconds) {
        long startTime = System.currentTimeMillis();
        long endTime = startTime + milliseconds;
        while(System.currentTimeMillis() <= endTime) {}
    }

    /** Waits for the amount of time in seconds specified before continuing.
     * @param milliseconds  the amount of time to wait in seconds.
    */
    public static void sleepSec(int seconds) {
        sleep(seconds * 1000);
    }
}