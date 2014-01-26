/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import javax.microedition.io.Connector;

/**
 *
 * @author kspoelstra
 */
public class Datalogger {

    private  PrintStream out = null;

    private static Datalogger instance = null;

    private Datalogger()
    {
        try{
            OutputStream os = Connector.openOutputStream("file:///Log.txt");
            out = new PrintStream(os);
        }catch(IOException ioe)
        {
            ioe.printStackTrace();
        }
    }

    public static Datalogger getInstance()
    {
        if(instance == null){
            instance = new Datalogger();
        }
        return instance;
    }


    public void print(String s)
    {
        out.print(s);
        out.flush();
    }

    public void println(String s)
    {
        out.println(s);
        out.flush();
    }
}
