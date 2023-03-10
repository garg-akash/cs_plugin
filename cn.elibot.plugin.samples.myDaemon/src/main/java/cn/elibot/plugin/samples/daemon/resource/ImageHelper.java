package cn.elibot.plugin.samples.daemon.resource;

import javax.swing.*;
import java.awt.*;
import java.net.URL;

public class ImageHelper {
    public static ImageIcon loadImage(String name) {
        ImageIcon image = null;

        try {
            URL url = ImageHelper.class.getResource("/images/icons/" + name);
            if (url != null) {
                Image img = Toolkit.getDefaultToolkit().createImage(url);
                if (img != null) {
                    image = new ImageIcon(img);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return image;
    }
}
