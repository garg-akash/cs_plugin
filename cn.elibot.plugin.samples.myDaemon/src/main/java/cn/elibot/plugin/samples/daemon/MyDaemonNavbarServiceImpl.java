package cn.elibot.plugin.samples.daemon;

import cn.elibot.plugin.samples.daemon.resource.ImageHelper;
import cn.elibot.plugin.samples.daemon.resource.ResourceSupport;
import cn.elibot.robot.plugin.contribution.navbar.NavbarContext;
import cn.elibot.robot.plugin.contribution.navbar.NavbarContribution;
import cn.elibot.robot.plugin.contribution.navbar.NavbarService;

public class MyDaemonNavbarServiceImpl implements NavbarService {
    private final MyDaemonServiceImpl daemonService;

    public MyDaemonNavbarServiceImpl(MyDaemonServiceImpl daemonService) {
        this.daemonService = daemonService;
    }

    @Override
    public void configure(NavbarContext navbarContext) {
        navbarContext.setNavbarIcon(ImageHelper.loadImage("small_plugin.png"));
        navbarContext.setNavbarName(ResourceSupport.getDefaultResourceBundle().getString("plugin_title"));
    }

    @Override
    public NavbarContribution createContribution() {
        return new MyDaemonViewImpl(this.daemonService);
    }
}
