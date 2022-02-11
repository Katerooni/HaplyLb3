//pebbles on a beach
//by ekaterina durmanova

//rocks large jagged motions
//sand pushing back but staticky feel?
//water/waves /slight pushing back


/* library imports *****************************************************************************************************/
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/

/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/

/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/

/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define game ball */
FCircle           g2;
FBox              g1;

/* define start and stop button */
FBox           b3;
FBox           b2;
FBox           b1;
//motion
FBox mo;
float wave = 25;
/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  size(1000, 400);

  /* device setup */

  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[2], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);


  widgetOne.device_set_parameters();

  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();

  /* Section1 */
  b1                  = new FBox(9.0, 9.0);
  b1.setPosition(3.5, edgeTopLeftY+worldHeight/2.0);
  b1.setFill(0, 0, 200);
  b1.setStaticBody(true);
  b1.setSensor(true);
  b1.setDensity(500);
  b1.setSensor(false);
  b1.setNoStroke();
  b1.setStatic(true);
  world.add(b1);

  /* Section2 */
  b2                  = new FBox(9, 9.0);
  b2.setPosition(worldWidth/2, worldHeight/2.0);
  b2.setFill(0, 0, 200);
  b2.setStaticBody(true);
  b2.setSensor(true);
  b2.setDensity(500);
  b2.setSensor(false);
  b2.setNoStroke();
  b2.setStatic(true);
  world.add(b2);


  /* Section3*/
  b3                  = new FBox(9, 9.0); // diameter is 2
  b3.setPosition(worldWidth-3.5, edgeTopLeftY+worldHeight/2.0);
  b3.setFill(0, 0, 200);
  b3.setStaticBody(true);
  b3.setDensity(500);
  b3.setSensor(false);
  b3.setNoStroke();
  b3.setStatic(true);
  world.add(b3);

  /* moving wave*/
  mo            = new FBox(2, 9.0);
  mo.setPosition(wave, edgeTopLeftY+worldHeight/2.0);
  mo.setFill(0, 0, 200);
  mo.setStaticBody(true);
  mo.setSensor(true);
  mo.setDensity(500);
  mo.setSensor(false);
  mo.setNoStroke();
  mo.setStatic(true);
  world.add(mo);

  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255, 0, 0); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);

  world.draw();

  /* setup framerate speed */
  frameRate(baseFrameRate);

  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}


/* end setup section ******************************
 ***************************************************
 **************************************************/



/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if (renderingForce == false) {
    background(255);
    if (wave>-2) {
      wave=wave-.1;
    } else if (wave<=-2) {
      wave=30;
    }
    mo.setPosition(wave, edgeTopLeftY+worldHeight/2.0);
 
    world.draw(); 

    println(wave);
  }
}
/* end draw section ****************************************************************************************************/


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;

    if (haplyBoard.data_available()) {
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();

      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));
    }

    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();


    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons


    /* Sand */
    if (s.h_avatar.isTouchingBody(b2)) {
      s.h_avatar.setSensor(true);

      s.h_avatar.setDamping(700);
    }

    /* Rocky Beach */
    if (s.h_avatar.isTouchingBody(b1)) {
      s.h_avatar.setSensor(true);
      s.h_avatar.setDamping(400);

      fEE.y = random(-2, 2);
      ;
    }
    //wavy water
    if (s.h_avatar.isTouchingBody(b3)) {
      s.h_avatar.setSensor(true);
      s.h_avatar.setDamping(200);
    }


    if (s.h_avatar.isTouchingBody(mo)) {
      s.h_avatar.setSensor(true);
      s.h_avatar.setDamping(950);
    }
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();



    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}

/* end simulation section **********************************************************************************************/
