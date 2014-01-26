/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author kspoelstra
 */
import com.sun.squawk.util.LineReader;
import edu.wpi.first.wpilibj.Timer;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Hashtable;
import java.util.TimerTask;
import javax.microedition.io.Connector;

/**
 * A useful class for adding dynamic settings to the FRC Java. Allows you to set
 * numbers which are to be dynamically loaded from a file names settings.txt on
 * the root of the cRIO. The is only one mandatory setting, which is
 * "Autoupdate Delay". This "Autoupdate Delay" setting determines how long in
 * milliseconds until the settings are automatically reloaded. If you have any
 * questions or comments you can contact me at th4019@gmail.com
 *
 * @author Thomas Hansen - Team 3006
 */
public class Settings
{
    static Hashtable hash;
    static long lastUpdateTime;

    private static void load() {
        //System.out.println("settings reloaded");
        if (hash == null) {
            hash = new Hashtable();
            java.util.Timer dashTimer = new java.util.Timer();
            dashTimer.schedule(new TimerTask() {

                public void run() {
                    long now = Timer.getUsClock();

                    if (now - lastUpdateTime > (Settings.getDouble("Autoupdate Delay") * 10E6)) {
                        load();
                        lastUpdateTime = now;
                    }
                }
            }, 0, 50);
        }

        try {
            InputStream is = Connector.openInputStream("file:///Settings.txt");

            LineReader r = new LineReader(new InputStreamReader(is));

            String line = null;

            while ((line = r.readLine()) != null) {
                int m = line.indexOf("=");
                if (m != -1) {
                    String key = line.substring(0, m - 1).trim();
                    String value = line.substring(m + 2, line.length()).trim();

                    //System.out.println("-"+key+"-"+value+"-");

                    hash.put(key, value);
                }
            }


            is.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /*
     * Same as getString but parse the String and returns a long
     */
    public static long getLong(String name) {
        return Long.parseLong(getString(name));
    }

    /*
     * Same as getString but parse the String and returns a boolean
     */
    public static boolean getBoolean(String name) {
        return getString(name).equals("true");
    }

    /*
     * Same as getString but parse the String and returns a double
     */
    public static double getDouble(String name) {
        return Double.parseDouble(getString(name));
    }

    /*
     * Same as getString but parse the String and returns a integer
     */
    public static int getInt(String name) {
        return Integer.parseInt(getString(name));
    }

    /**
     *
     * @param name The "key" of the settings to be reloaded.
     * @return The value currently associated with the given key.
     */
    public static String getString(String name) {
        if (hash == null) {
            load();
        }
        return (String) hash.get(name);
    }
}

