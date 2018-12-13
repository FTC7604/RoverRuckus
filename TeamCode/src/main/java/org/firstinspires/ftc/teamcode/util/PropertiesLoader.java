package org.firstinspires.ftc.teamcode.util;

import android.content.Context;
import android.content.res.Resources;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

public class PropertiesLoader
{
    private Properties properties;

    public PropertiesLoader(String fileName)
    {
        File sdcard = Environment.getExternalStorageDirectory();
        File config = new File(sdcard, "DWAIConfig/" + fileName + ".properties");
        properties = new Properties();
        try
        {
            properties.load(new FileInputStream(config));
        } catch (IOException e)
        {
            throw new RuntimeException(e);
        }
    }

    public int getIntegerProperty(String name)
    {
        return Integer.parseInt(getStringProperty(name));
    }

    public float getFloatProperty(String name)
    {
        return Float.parseFloat(getStringProperty(name));
    }

    public double getDoubleProperty(String name)
    {
        return Double.parseDouble(getStringProperty(name));
    }

    public String getStringProperty(String name)
    {
        return properties.getProperty(name);
    }
}
