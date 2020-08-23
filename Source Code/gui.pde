/* ========================================================= //<>// //<>//
 * ====                   WARNING                        ===
 * =========================================================
 * The code in this tab has been generated from the GUI form
 * designer and care should be taken when editing this file.
 * Only add/edit code inside the event handlers i.e. only
 * use lines between the matching comment tags. e.g.

 void myBtnEvents(GButton button) { //_CODE_:button1:12356:
     // It is safe to enter your event code here  
 } //_CODE_:button1:12356:
 
 * Do not rename this tab!
 * =========================================================
 */

public void panel1_Click1(GPanel source, GEvent event) { //_CODE_:panel1:951333:
  println("panel1 - GPanel >> GEvent." + event + " @ " + millis());
} //_CODE_:panel1:951333:

public void panel2_Click1(GPanel source, GEvent event) { //_CODE_:panel2:919711:
  println("panel2 - GPanel >> GEvent." + event + " @ " + millis());
} //_CODE_:panel2:919711:

public void button1_click1(GButton source, GEvent event) { //_CODE_:button1:277440:
  println("button1 - GButton >> GEvent." + event + " @ " + millis());
  
} //_CODE_:button1:277440:

public void button2_click1(GButton source, GEvent event) { //_CODE_:button2:933335:
  println("button2 - GButton >> GEvent." + event + " @ " + millis());
   if (!l.isEmpty()) {
      l.remove(l.size()-1);
      
      
    }
} //_CODE_:button2:933335:

public void button3_click1(GButton source, GEvent event) { //_CODE_:button3:295639:
  println("button3 - GButton >> GEvent." + event + " @ " + millis());
  l.clear();
} //_CODE_:button3:295639:

public void button4_click1(GButton source, GEvent event) { //_CODE_:button4:821338:
  println("button4 - GButton >> GEvent." + event + " @ " + millis());
   exit();
} //_CODE_:button4:821338:



// Create all the GUI controls. 
// autogenerated do not edit
public void createGUI(){
  G4P.messagesEnabled(false);
  G4P.setGlobalColorScheme(GCScheme.BLUE_SCHEME);
  G4P.setMouseOverEnabled(true);
  G4P.setDisplayFont("Arial", G4P.BOLD, 60);
  G4P.setInputFont("Arial", G4P.BOLD, 35);
  G4P.setSliderFont("Arial", G4P.BOLD, 35);


  surface.setTitle("Sketch Window");
  
  panel1 = new GPanel(this, -2350,2100,1200,200, "Tab bar text");
  panel1.setText("Tab bar text");
  panel1.setLocalColorScheme(GCScheme.ORANGE_SCHEME);
  panel1.setOpaque(true);
  panel1.addEventHandler(this, "panel1_Click1");
  
  label1 = new GLabel(this,20,20,1000,200);
  label1.setTextAlign(GAlign.CENTER, GAlign.MIDDLE);
  label1.setText("My label");
  label1.setOpaque(false);
  panel1.addControl(label1);
  
  panel2 = new GPanel(this, -2305,-2305,4800,250, "Tab bar text");
  panel2.setText("Tab bar text");
  panel2.setLocalColorScheme(GCScheme.ORANGE_SCHEME);
  panel2.setOpaque(false);
  panel2.addEventHandler(this, "panel2_Click1");
  
  button1 = new GButton(this, 20, 21, 200,200);
  button1.setText("Save");
  button1.setLocalColorScheme(GCScheme.ORANGE_SCHEME);
  button1.addEventHandler(this, "button1_click1");
  
  button2 = new GButton(this, 260, 21, 200, 200);
  button2.setText("Undo");
  button2.setLocalColorScheme(GCScheme.ORANGE_SCHEME);
  button2.addEventHandler(this, "button2_click1");
  

  
  button3 = new GButton(this, 500,21, 200, 200);
  button3.setText("Delete");
  button3.setLocalColorScheme(GCScheme.ORANGE_SCHEME);
  button3.addEventHandler(this, "button3_click1");
  
  button4 = new GButton(this, 4400, 21, 200,200);
  button4.setText("Exit");
  button4.setLocalColorScheme(GCScheme.ORANGE_SCHEME);
  button4.addEventHandler(this, "button4_click1");
  
  panel2.addControl(button1);
  panel2.addControl(button2);
  panel2.addControl(button3);
  panel2.addControl(button4);
  
  
}
void display_coordinates()
{
  label1.setText(" POINT AT ( " + x + " , " + y +" , " + z+ " ) " );
  
}

// Variable declarations 
// autogenerated do not edit
GPanel panel1; 
GLabel label1; 
GPanel panel2; 
GButton button1; 
GButton button2; 
GButton button3; 
GButton button4; 