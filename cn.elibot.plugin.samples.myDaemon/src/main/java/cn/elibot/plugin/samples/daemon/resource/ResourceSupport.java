package cn.elibot.plugin.samples.daemon.resource;

import cn.elibot.robot.commons.lang.resource.LocaleProvider;
import cn.elibot.robot.commons.lang.resource.UTF8Control;

import java.util.Locale;
import java.util.ResourceBundle;

public class ResourceSupport {
    private static LocaleProvider provider;
    private static final String PROPERTIES = "i18n.text";

    public static void setLocaleProvider(LocaleProvider provider) {
        ResourceSupport.provider = provider;
    }

    public static LocaleProvider getLocaleProvider() {
        return ResourceSupport.provider;
    }

    public static ResourceBundle getResourceBundle(Locale locale) {
        return ResourceBundle.getBundle(PROPERTIES, locale, new UTF8Control());
    }

    public static ResourceBundle getDefaultResourceBundle() {
        return ResourceBundle.getBundle(PROPERTIES, provider.getLocale(), new UTF8Control());
    }
}
