package gui;

import javax.swing.AbstractButton;
import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JFrame;
import javax.swing.JOptionPane;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
 
public class ButtonWindow extends JPanel
                        implements ActionListener {
	private static final long serialVersionUID = 1L;
	protected JButton button1, button2, button3, button4;
	private static Graphics graphics;
 
    private ButtonWindow() {
 
        button1 = new JButton("drone view");
        button1.setVerticalTextPosition(AbstractButton.CENTER);
        button1.setHorizontalTextPosition(AbstractButton.LEADING);
        button1.setMnemonic(KeyEvent.VK_1);
        button1.setActionCommand("DRONE_CAM");
 
        button2 = new JButton("chase view");
        button2.setMnemonic(KeyEvent.VK_2);
        button2.setActionCommand("DRONE_CHASE_CAM");
        
        button3 = new JButton("top down view");
        button3.setMnemonic(KeyEvent.VK_3);
        button3.setActionCommand("DRONE_TOP_DOWN_CAM");
        
        button4 = new JButton("side view");
        button4.setMnemonic(KeyEvent.VK_4);
        button4.setActionCommand("DRONE_SIDE_CAM");
 
        button1.setBackground(Color.CYAN);
        button2.setBackground(Color.lightGray);
        button3.setBackground(Color.lightGray);
        button4.setBackground(Color.lightGray);
        
        button1.addActionListener(this);
        button2.addActionListener(this);
        button3.addActionListener(this);
        button4.addActionListener(this);
 
        button1.setToolTipText("Shows the drone from its perspective.");
        button2.setToolTipText("Shows the drone from behind.");
        button3.setToolTipText("Shows the drone from the top.");
        button4.setToolTipText("Shows the drone from the side.");
 
        add(button1);
        add(button2);
        add(button3);
        add(button4);
    }
 
    public void actionPerformed(ActionEvent e) {
    	if ("DRONE_CAM".equals(e.getActionCommand())) {
    		button1.setBackground(Color.CYAN);
            button2.setBackground(Color.lightGray);
            button3.setBackground(Color.lightGray);
            button4.setBackground(Color.lightGray);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_CAM);
    			break;
        	}
        } else if ("DRONE_CHASE_CAM".equals(e.getActionCommand()))  {
        	button1.setBackground(Color.lightGray);
            button2.setBackground(Color.CYAN);
            button3.setBackground(Color.lightGray);
            button4.setBackground(Color.lightGray);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_CHASE_CAM);
    			break;
        	}
        } else if ("DRONE_TOP_DOWN_CAM".equals(e.getActionCommand()))  {
        	button1.setBackground(Color.lightGray);
            button2.setBackground(Color.lightGray);
            button3.setBackground(Color.CYAN);
            button4.setBackground(Color.lightGray);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_TOP_DOWN_CAM);
    			break;
        	}
        } else if ("DRONE_SIDE_CAM".equals(e.getActionCommand()))  {
        	button1.setBackground(Color.lightGray);
            button2.setBackground(Color.lightGray);
            button3.setBackground(Color.lightGray);
            button4.setBackground(Color.CYAN);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_SIDE_CAM);
    			break;
        	}
        }
    }

    public static void createAndShowWindow(Graphics graphics) {
 
    	ButtonWindow.graphics = graphics;
    	
        JFrame frame = new JFrame("Drone command");
        frame.setLocation(-10, 0);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
 
        ButtonWindow newContentPane = new ButtonWindow();
        newContentPane.setOpaque(true);
        frame.setContentPane(newContentPane);
 
        frame.pack();
        frame.setVisible(true);
    }
}


