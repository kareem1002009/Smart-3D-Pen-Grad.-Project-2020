///////////////////////////////////////////////////////////////////////////////////////// //<>//
////////////////////////////////////////// SMART 3D PEN//////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

/************************************************Describtion*******************************************************
       This project to design a smart 3-D pen that will assist the Engineering drawing with handy ,smart
       and easy to use tool. the movements of the engineer in free space will automatically be converted
       to a three dimentional drawing in the software application.
*******************************************************************************************************************/

import g4p_controls.*;
import nervoussystem.obj.*;
import peasy.*;
import oscP5.*;
import netP5.*;

PeasyCam cam;
OscP5 oscP5;                                                     //OSC object

boolean record = false;                                         //Check Save
String data;
float x, y, z, s3, s1, s2, GUI_flag; 
int gridSize = 10 ;                                              //Grid cell Size
int over_save=0;                                                //Over Save Flag 
int over_undo=0;                                                //Over Undo Flag
int over_delete=0;                                              //Over Delete Flag
int over_exit=0;                                                //Over Exit Flag                                       
String hand;                                                    //Check Right/Left Hand
int ID=0;
float Radius = 250.0;                                           //Zoom Parameter
int motion_flag;                                                //Check Motion Mode
float camHeight = 0;                                            //Camera 
int scene_XValue = 200;                                         //Camera Parameter
int scene_YValue = 200;                                         //Camera Parameter
int scene_ZValue = 500;                                         //Camera Parameter
 
ArrayList <String> l = new ArrayList <String> ();                                        //List Of Points
ArrayList<ArrayList<Float>> array_x = new ArrayList<ArrayList<Float>>();                 //2D Array Of X_object
ArrayList<ArrayList<Float>> array_y = new ArrayList<ArrayList<Float>>();                 //2D Array Of Y_object
ArrayList<ArrayList<Float>> array_z = new ArrayList<ArrayList<Float>>();                 //2D Array Of Z_object
 
public void setup() {

  cam = new PeasyCam(this, 4000);                                                       //Camera Object
  size(700, 700, P3D);                                                                  //Sketch Size
  cursor(CROSS);
  background(0);                                                                       //Set Outlines Of All Drawing to be White
  stroke(255);
  oscP5 = new OscP5(this, 12000);                                                     //Identify Port Number
  createGUI();                                                                        //GUI Function
  customGUI();                                                                        //GUI Function
}//End Setup 

void draw() 
{
  background(0);
  grid_3d();                                                                         //Call To Draw 3D Grid
  display_coordinates();                                                             //Display Coordinates On The Box  

  if (over_save==1)                                                                  
  {
    record=true;                                                                     //Set To Start Save Objects
  }
  
  if(!(l.isEmpty()))
  {
    pointer();                                                                      //Call Display Pointer Over Point Location
  }

  for ( int index = 0; index < l.size(); index++)                                   // Iterate Over The Data Points List 
  {
    String[] split_vec = split(l.get(index), ' ');                                  // Split Eeach Line Of Data
    
    if (split_vec.length==3)                                                        // Zoom Mode 
    {
      float zoom=float(split_vec[1]);
      Radius+=zoom*2;
      if (Radius<-2) 
      {
        Radius=-2;
      }
      float angle = map(20, 0, width, 360, 0);
      camHeight = map(20, 0, height, height+400, 0);
      PVector camPos = new PVector();
      camPos.x = Radius*sin(radians(angle)) + scene_XValue;
      camPos.y = camHeight;
      camPos.z = Radius*cos(radians(angle)) + scene_ZValue;
      camera (
        camPos.x, camPos.y, camPos.z, 
        scene_XValue, scene_YValue, scene_ZValue, 
        0.0, 1.0, 0.0
        );
      l.remove(l.size()-1);
    }// End Zoom Mode
    
   else if (split_vec.length==6 )                                                   // Camera Mode 
    {       
      motion_flag=int(split_vec[1]);  
      if (motion_flag==1 )
      {
        float angle = map(float(split_vec[2]), 0, width, 360, 0);
        camHeight = map(float(split_vec[3]), 0, height, height+400, 0);

        PVector camPos = new PVector();
        camPos.x = Radius*sin(radians(angle)) + scene_XValue;
        camPos.y = camHeight;
        camPos.z = Radius*cos(radians(angle)) + scene_ZValue;
        camera (
          camPos.x, camPos.y, camPos.z, 
          scene_XValue, scene_YValue, scene_ZValue, 
          0.0, 1.0, 0.0
          );
        l.remove(l.size()-1);
      } 
    }// End Camera Mode
    
    else if (split_vec.length==9 )                                                      //pen Mode
      {
          x = float(split_vec[5])   ;
          y = float(split_vec[4])   ;
          z = float(split_vec[6])   ;
          GUI_flag=float(split_vec[7])  ;

    /*      x = map(x, -3500, 3500, -width, width);
            y = map(y,  -3500, 3500 , -width, width);
            z = map(z,  -600,600 , -width, width);
           print("x = ",x," y = ",y," z = ",z,"\n");*/
           
          if (GUI_flag==1)                                                              // GUI Mode
          {
            if ((x>=-80 && x<=120)&&(y>=-79 && y<=121))
            {
              over_save=1;
              over_undo=0;
              over_delete=0;
              over_exit=0;
            } 
            else if ((x>=160 && x<=360)&&(y>=-79 && y<=121))
            {
              over_save=0;
              over_undo=1;
              over_delete=0;
              over_exit=0;
            } 
            else if ((x>=400 && x<=600)&&(y>=-79 && y<=121))
            {
              over_save=0;
              over_undo=0;
              over_delete=1;
              over_exit=0;
            } 
            else if ((x>=4300 && x<=4500)&&(y>=-79 && y<=121))
            {
              over_save=0;
              over_undo=0;
              over_delete=0;
              over_exit=1;
            }

            l.remove(l.size()-1);
            if (over_save==1)
            {
              record = true;
              save_fun();
            }
            else if ( over_undo==1)
            {
              button2_click1(button2, GEvent.CLICKED);
              print(l);
            } 
            else if (over_delete==1)
            { 
              button3_click1(button3, GEvent.CLICKED);
              print(l);
            } 
            else if (over_exit==1)
            { 
              button4_click1(button4, GEvent.CLICKED);
              print(l);
            }
          }// End Gui Mode
                 
        else if (float(split_vec[2]) == 1 && float(split_vec[3]) == 0)                      //Drawing  
        {
             if (index > 0 &&float(split(l.get(index-1), ' '))[2]==1) 
            {
               stroke(255);
               strokeWeight(4);
               line ( x, y, z, s1, s2, s3);
            }
          s1 = x;
          s2 = y;
          s3 = z;
        }// End Drawing
       else if (float(split_vec[2]) == 0 && float(split_vec[3]) == 1)                       // Erase   
        {
          erase(x,y,z);
        }//End Erase 
      }//End Pen Mode
    }//End For Loop
}//End Draw

/*Over Write The Point Location With Black Sphere*/
void erase (float a, float b, float c)
{
  noStroke();          // Setting the outline (stroke) to black
  fill(0, 0, 0); 
  pushMatrix();
  translate(a, b, c);// fill the circle with black 
  sphere  ( 10);  //  2r = 20
  popMatrix();
}// End Erase Func

/*Function To Draw 2D Grid */
void grid ()
{
  pushStyle () ;
  strokeWeight (1);
  stroke(0, 51, 153);
  for (int i = -width/5; i <= width/5; i+=gridSize) {
    for (int j = -height/5; j <= height/5; j+=gridSize) {
      line(i, j, i, height/5);
      line(i, j, width/5, j);
    }
  }
  popStyle () ;
}// End 2D Grid Func

/*Function To Draw 3D Grid*/
void grid_3d ()
{
  pushMatrix();
  background(0);
  grid();
  translate (-width/5, 0, width/5);
  rotateY(-PI/2);
  grid();
  translate (0, -height/5, -height/5);
  fill(255, 0, 0);
  rotateX(PI/2);
  grid();
  popMatrix();
}// End 3D Grid Func

/*Function Of Reciever Data*/
void oscEvent(OscMessage message)
{
  if(message.checkAddrPattern("/data")==true)
  {
    if(message.checkTypetag("s")) {
      data = message.get(0).stringValue();
      l.add(data);
      println(data);
    }
  }
  else if(message.checkAddrPattern("/reset")==true)
  {
    if(message.checkTypetag("s")) {
      l.clear();
      array_x.clear();
      array_y.clear();
      array_z.clear();
    }
  } 
}// End Reciever Func

/*Control GUI Using Keyboard*/
void keyPressed() {
  switch (key) {
  case BACKSPACE:
      if (!l.isEmpty()) {
      l.remove(l.size()-1);
    }
    break;
  case 'd':
    l.clear();
    array_x.clear();
    array_y.clear();
    array_z.clear();
    break;
  case 'q':
    exit();
    break;
  case's' :
    record = true;
    save_fun();
    ID++;
    break;
  default:
    break;
  } //End Switch
} // End Func

/*Control GUI Using Mouse Click*/
void mousePressed()
{
  if((mouseX>=5 && mouseX<=31)&&(mouseY>=4 && mouseY<=31))
   {
       record = true;
       save_fun();
       ID++;
   }
  else if((mouseX>=42 && mouseX<=68)&&(mouseY>=5 && mouseY<=31))
   {
      if (!l.isEmpty()) 
      {
          l.remove(l.size()-1);
      }
   }
  else if((mouseX>=78 && mouseX<=103)&&(mouseY>=3 && mouseY<=31))
   {
       l.clear();
       array_x.clear();
       array_y.clear();
       array_z.clear();
   }
  else if((mouseX>=669 && mouseX<=697)&&(mouseY>=6 && mouseY<=31))
   {
       exit();
   }

}// End Func

/*Function To Save All Drawing As OBJ File*/
void save_fun()
{
  int count=0;
  if (record) {
    
    array_x=object_count(5);
    array_y=object_count(4);
    array_z=object_count(6);
    beginRecord("nervoussystem.obj.OBJExport", "at"+ID+".obj");

   for (int i=0; i<array_x.size(); i++)                                                              //Iterate Over Number Of Objects
    {
      count=0;
      beginShape();
      while (count<array_x.get(i).size())
      {
        vertex(array_x.get(i).get(count), array_y.get(i).get(count), array_z.get(i).get(count));     //Iterate Over Object Points
        count++;
      }
      endShape();
    }
     endRecord();
     record = false;                                                                                //TO Stop Saving
  } 
}// End Save Func

/*Function To Seperate Different Objects*/
ArrayList<ArrayList<Float>> object_count(int p)
{
  ArrayList<Float>array_temp=new ArrayList<Float>();
  ArrayList<ArrayList<Float>>result=new ArrayList<ArrayList<Float>>();

  for (int i=0; i<l.size(); i++)
  {
    if (float(split(l.get(i), ' '))[2]==1)
    {
      array_temp.add(float(split(l.get(i), ' '))[p]);
    }

    else if (i!=0 &&float(split(l.get(i), ' '))[2]==0&&float(split(l.get(i-1), ' '))[2]==1)
    { 
      result.add(new ArrayList<Float> (array_temp));
      array_temp.clear();
    }
  }
  
  return result;                                                                         //List To Save Seperated Objects
}

/*Function To Display Pointer Over Recieved Point Location*/
void pointer()
{
  String[] point_pleace=split(l.get(l.size()-1), ' ');
  stroke(125);          
  fill(125); 
  pushMatrix();
  translate(float(point_pleace[5]),float(point_pleace[4]),float(point_pleace[6]));
  sphere (2.5); 
  popMatrix();
  fill(255);
  textSize(10);
  text("X: "+float(point_pleace[4])+"Y: "+float(point_pleace[5])+"z: "+float(point_pleace[6]), float(point_pleace[5])+10, float(point_pleace[5])-10, float(point_pleace[6])-10);
}
public void customGUI()
{
}                  
