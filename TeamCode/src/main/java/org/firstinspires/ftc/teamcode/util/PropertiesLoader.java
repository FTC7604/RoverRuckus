package org.firstinspires.ftc.teamcode.util;

import android.content.Context;
import android.content.res.Resources;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

public class PropertiesLoader
{
    private Context context;
    private int resource;

    public PropertiesLoader(Context context, int resource)
    {
        this.context = context;
        this.resource = resource;
    }

    public PropertiesLoader(HardwareMap map, int resource)
    {
        this(map.appContext, resource);
    }

    public PropertiesLoader(OpMode op, int resource)
    {
        this(op.hardwareMap, resource);
    }

    public String getConfigValue(String name)
    {
        Resources resources = context.getResources();

        try
        {
            InputStream rawResource = resources.openRawResource(resource);
            Properties properties = new Properties();
            properties.load(rawResource);
            return properties.getProperty(name);
        } catch (Resources.NotFoundException | IOException e)
        {
            throw new RuntimeException(e);
        }
    }
}
