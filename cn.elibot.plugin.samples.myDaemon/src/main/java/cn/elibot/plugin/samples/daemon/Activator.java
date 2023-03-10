package cn.elibot.plugin.samples.daemon;

import cn.elibot.plugin.samples.daemon.resource.ResourceSupport;
import cn.elibot.robot.commons.lang.resource.LocaleProvider;
import cn.elibot.robot.plugin.contribution.daemon.DaemonService;
import cn.elibot.robot.plugin.contribution.navbar.NavbarService;
import org.osgi.framework.BundleActivator;
import org.osgi.framework.BundleContext;
import org.osgi.framework.ServiceReference;
import org.osgi.framework.ServiceRegistration;

public class Activator implements BundleActivator {
    private ServiceRegistration<DaemonService> registration;
    private ServiceReference<LocaleProvider> localeProviderServiceReference;

    @Override
    public void start(BundleContext bundleContext) throws Exception {
        localeProviderServiceReference = bundleContext.getServiceReference(LocaleProvider.class);
        if (localeProviderServiceReference != null) {
            LocaleProvider localeProvider = bundleContext.getService(localeProviderServiceReference);
            if (localeProvider != null) {
                ResourceSupport.setLocaleProvider(localeProvider);
            }
        }
        System.out.println("cn.elibot.plugin.example.myDaemon.impl.Activator says Hello World!");
        MyDaemonServiceImpl daemonService = new MyDaemonServiceImpl();
        registration = bundleContext.registerService(DaemonService.class, daemonService, null);
        bundleContext.registerService(NavbarService.class, new MyDaemonNavbarServiceImpl(daemonService), null);
    }

    @Override
    public void stop(BundleContext bundleContext) throws Exception {
        System.out.println("cn.elibot.plugin.example.myDaemon.impl.Activator says Goodbye World!");
        this.registration.unregister();
    }
}
