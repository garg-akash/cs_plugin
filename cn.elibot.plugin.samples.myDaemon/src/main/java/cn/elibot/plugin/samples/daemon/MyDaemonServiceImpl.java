package cn.elibot.plugin.samples.daemon;

import cn.elibot.robot.plugin.contribution.daemon.DaemonContribution;
import cn.elibot.robot.plugin.contribution.daemon.DaemonService;

import java.net.MalformedURLException;
import java.net.URL;

public class MyDaemonServiceImpl implements DaemonService {

    private DaemonContribution daemonContribution;

    @Override
    public void init(DaemonContribution contribution) {
        this.daemonContribution = contribution;
        try {
            daemonContribution.installResource(new URL("file:daemon/"));
        } catch (MalformedURLException ignore) {
        }
        //自动启动
//        daemonContribution.start();
    }

    @Override
    public URL getExecutable() {
        try {
//            return new URL("file:daemon/server.py");
            return new URL("file:daemon/HelloWorld");
        } catch (MalformedURLException e) {
            return null;
        }
    }

    public DaemonContribution getDaemon() {
        return daemonContribution;
    }
}
