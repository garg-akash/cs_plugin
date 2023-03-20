package cn.elibot.plugin.samples.daemon;

import cn.elibot.plugin.samples.daemon.resource.ResourceSupport;
import cn.elibot.robot.plugin.contribution.daemon.DaemonContribution;
import cn.elibot.robot.plugin.contribution.navbar.NavbarContribution;
import cn.elibot.robot.plugin.ui.SwingService;
import cn.elibot.robot.plugin.ui.model.BaseKeyboardCallback;
import org.apache.xmlrpc.XmlRpcException;

import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.text.DefaultFormatterFactory;
import javax.swing.text.NumberFormatter;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.text.NumberFormat;

public class MyDaemonViewImpl implements NavbarContribution {

    private final MyDaemonServiceImpl daemonService;
    private final XmlRpcMyDaemonFacade xmlRpcMyDaemonFacade;
    private JButton startButton;
    private JButton stopButton;
    private JTextField textField;
    private JButton moveitButton;
    private JFormattedTextField textField_time;
    private JButton deployTrajButton;

    public MyDaemonViewImpl(MyDaemonServiceImpl daemonService) {
        this.daemonService = daemonService;
        this.xmlRpcMyDaemonFacade = new XmlRpcMyDaemonFacade("127.0.0.1", 4444);
    }

    @Override
    public void buildUI(JPanel panel) {

        panel.setBackground(Color.WHITE);
        panel.setLayout(new GridBagLayout());

        JLabel label = new JLabel(ResourceSupport.getDefaultResourceBundle().getString("daemon_view"));
        textField = new JTextField("/move_group/display_planned_path");
        textField.setPreferredSize(new Dimension(200, 32));
        startButton = new JButton(ResourceSupport.getDefaultResourceBundle().getString("start_daemon"));
        stopButton = new JButton(ResourceSupport.getDefaultResourceBundle().getString("stop_daemon"));
        moveitButton = new JButton("Interpolate");
        deployTrajButton = new JButton("Deploy");

        NumberFormat customFormat = NumberFormat.getNumberInstance();
        customFormat.setRoundingMode(RoundingMode.UNNECESSARY);
        customFormat.setMaximumFractionDigits(3);
        textField_time = new JFormattedTextField(new NumberFormatter(customFormat));


        textField.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                SwingService.keyboardService.showLetterKeyboard(textField, null, false, new BaseKeyboardCallback() {
                    @Override
                    public void onOk(Object o) {
                        try {
                            xmlRpcMyDaemonFacade.setMessage(textField.getText());
                        } catch (XmlRpcException ex) {
                            ex.printStackTrace();
                        }
                    }
                });
            }
        });

        startButton.addActionListener(e -> startDaemon());

        stopButton.addActionListener(e -> stopDaemon());

        moveitButton.addActionListener(e -> moveitDaemon());
        deployTrajButton.addActionListener(e -> deployTrajDaemon());
        textField_time.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                SwingService.keyboardService.showNumberKeyboard(textField_time, 0.1, 0, 1, 2, new BaseKeyboardCallback() {
                    @Override
                    public void onOk(Object o) {
                        try {
                            xmlRpcMyDaemonFacade.setSamplingTime(Double.parseDouble(textField_time.getText()));
                        } catch (XmlRpcException ex) {
                            ex.printStackTrace();
                        }
                    }
                });
            }
        });

        addComponent(panel, label, 0, 0, 2, 1, 0.0D, 0.0D, 0, 0, 0, 0, 1, 10);
        addComponent(panel, textField, 0, 1, 2, 1, 0.0D, 0.0D, 20, 0, 0, 0, 1, 10);
        addComponent(panel, textField_time, 0, 2, 2, 1, 0.0D, 0.0D, 20, 0, 0, 0, 1, 10);
        addComponent(panel, startButton, 0, 3, 1, 1, 0.0D, 0.0D, 20, 0, 0, 0, 3, 17);
        addComponent(panel, stopButton, 1, 3, 1, 1, 0.0D, 0.0D, 20, 0, 0, 0, 3, 13);
        addComponent(panel, moveitButton, 0, 4, 1, 1, 0.0D, 0.0D, 20, 0, 0, 0, 3, 17);
        addComponent(panel, deployTrajButton, 1, 4, 1, 1, 0.0D, 0.0D, 20, 0, 0, 0, 3, 17);
    }

    public static void addComponent(JPanel panel, Component c, int x, int y, int width, int height, double wx, double wy, int top, int left, int bottom, int right, int fill, int anchor) {
        GridBagConstraints gridBagConstraints = new GridBagConstraints();
        gridBagConstraints.gridx = x;
        gridBagConstraints.gridy = y;
        gridBagConstraints.gridwidth = width;
        gridBagConstraints.gridheight = height;
        gridBagConstraints.weightx = wx;
        gridBagConstraints.weighty = wy;
        gridBagConstraints.anchor = anchor;
        gridBagConstraints.fill = fill;
        gridBagConstraints.insets = new Insets(top, left, bottom, right);
        panel.add(c, gridBagConstraints);
    }

    public void updateStatus() {
        if (this.daemonService.getDaemon().getState() == DaemonContribution.State.RUNNING) {
            startButton.setEnabled(false);
            stopButton.setEnabled(true);
            textField.setEnabled(true);
            moveitButton.setEnabled(true);
            deployTrajButton.setEnabled(true);
            textField_time.setEnabled(true);
        } else {
            startButton.setEnabled(true);
            stopButton.setEnabled(false);
            textField.setEnabled(false);
            moveitButton.setEnabled(true);
            deployTrajButton.setEnabled(true);
            textField_time.setEnabled(true);
        }
    }

    public void startDaemon() {
        System.out.println("my daemon start!");
        daemonService.getDaemon().start();
        updateStatus();
    }

    public void stopDaemon() {
        System.out.println("my daemon stop!");
        daemonService.getDaemon().stop();
        updateStatus();
    }

    public void moveitDaemon() {
        System.out.println("my daemon MoveIt!");
        try {
            xmlRpcMyDaemonFacade.getMoveitTraj(textField.getText());
        } catch (XmlRpcException ex) {
            ex.printStackTrace();
        }
//        daemonService.getDaemon().start();
//        updateStatus();
    }
    public void deployTrajDaemon() {
        System.out.println("my daemon Deploy!");
        try {
            xmlRpcMyDaemonFacade.deployMoveitTraj();
        } catch (XmlRpcException ex) {
            ex.printStackTrace();
        }
    }

    @Override
    public void openView() {
        updateStatus();
        startDaemon();
    }

    @Override
    public void closeView() {
        stopDaemon();
    }
}