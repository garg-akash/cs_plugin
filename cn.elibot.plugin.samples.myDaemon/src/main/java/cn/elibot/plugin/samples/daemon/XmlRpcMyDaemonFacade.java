package cn.elibot.plugin.samples.daemon;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;

public class XmlRpcMyDaemonFacade {
    private final XmlRpcClient client;

    public XmlRpcMyDaemonFacade(String host, int port) {
        XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
        config.setEnabledForExtensions(true);
        try {
            config.setServerURL(new URL("http://" + host + ":" + port + "/RPC2"));
        } catch (MalformedURLException e) {
            e.printStackTrace();
        }
        //1s
        config.setConnectionTimeout(1000);
        client = new XmlRpcClient();
        client.setConfig(config);
    }


    public String setMessage(String mes) throws XmlRpcException {
        ArrayList<String> args = new ArrayList<String>();
        args.add(mes);
        Object result = client.execute("set_message", args);
        return processString(result);
    }

    private String processString(Object response) {
        if (response instanceof String) {
            return (String) response;
        } else {
            return "";
        }
    }
}
