package graphPackage;


import java.awt.Color;
import java.awt.*;

import java.lang.Object;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;

import me.jjfoley.gfx.GFX;
import me.jjfoley.gfx.IntPoint;
import me.jjfoley.gfx.TextBox;

/**
 * a pretty simple object that just draws a box with the button's label
 * and can tell you if the button has been clicked
 * @author lucygroves
 *
 */
public class button {
	String name;
	Rectangle2D.Double box;
	
	
	button(String name, double width, double height, Point location){
		this.name = name;
		box = new Rectangle2D.Double(location.getX(), location.getY(), width, height);
	}
	
	void draw(Graphics2D g, Color color) {
		g.setColor(color);
		g.fillRect((int) box.getX(), (int) box.getY(), (int) box.width, (int) box.height);
		TextBox text = new TextBox(this.name);
		text.setColor(new Color(0, 0, 0));
		text.centerInside(box);
		text.draw(g);
	}
	
	boolean clicked(IntPoint point) {
		if (point!= null) {
			//System.out.println(click);
			if (box.contains(point)){
				//System.out.println(click);
				return true;
				
			}
		}
		return false;
		
		
	}
	
	
	
	

}
