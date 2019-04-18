// Shawn Daniel  //<>//
// 8/27/2018




// PARAMETERS TO TUNE PERFORMANCE
// thrMagn==2675 [lbs] // Maximum Thrust amount
float burnTime=3.0; // duration of constant thrust, [secs]
float ballistTime=1.0; // One second after launch, msl control system comes on, to actuate Msl



import peasy.*; 
import controlP5.*;  
import oscP5.*; 
import netP5.*; 


float MAX_TRANSLATION = 1; 
float MAX_ROTATION = 1;    
float dt1= .05;//.05;  // Discrete cycle time [secs]
boolean llave;
float tsec=0;
boolean guidFlag=false;
int guidFlagInt=0;


// DYNAMICS PARAMETERS
float g = 9.81; //  Gravity

// Masses and Weights and Moments  of Inertia
float mTot= 45.5 ;   // Total A/C mass [kg]
float weight= g*mTot  ;   
float Jx=1866/1, Jy=80, Jz=Jy;   // Moment of inertia,  originally at 80
// Maximum mechanical fin deflection.
float maxDeflection = 20.0*PI/180.0; 
// Msl fuseloge and static fin reaction drag forces
PVector[] Rwgrad, RwgradLado, Rwgradr, Rwgradl, Rwgradn, PosVec;
PVector[] Rwgradrf, Rwgradlf;
// Actuator fin reaction drag forces
PVector[] Rwgrad1, Rwgrad2, Rwgrad3, Rwgrad4;
// Msl fuseloge and static fin gradient vectors
PVector[] Grad, GradLado, Gradr, Gradl, Gradn;
// Actuator fin gradient Vectors
PVector[] Gradd1, Gradd2, Gradd3, Gradd4;
float kxx, kyy, kzz;
float xcomp, ycomp, zcomp; 
PVector[]  rdf1, rdf2, rdf3, rdf4; 
PVector[]  rdf1d, rdf2d, rdf3d, rdf4d;
PVector[]  rdf1f, rdf2f, rdf3f, rdf4f;
PVector[] Mf1, Mf2, Mf3, Mf4; 
PVector[] Mff1, Mff2, Mff3, Mff4; 
PVector[] Mfd1, Mfd2, Mfd3, Mfd4;
PVector[] MfIN;
// Thrust vector
PVector[] th;

// 3D PNG fins
PVector[] ZEM, ZEMparr, ZEMperp;


float thrx, thry, thrz, thrMagn;

float counter=0; 
float DdotG;
float PosVecNorm;


// Msl Design Parameters
 // Look in ACDragReactForce() Object
float inToMeters=  .0254;
float LengthFuse = 1.6256; //[m]    64"
float rho_sl=1.225;  // [kg/m^3]
float radius= 3.5*.0254; //[m]
float AreaNose= PI*radius*radius ; // max for  gradient of face paralel to gravity.
float AreaFuse=radius*2*1.6256;
float Areafwset = 1*(12.5*3*.0254*.0254)*2; // Area of the pair of fixed wing sets. TIMES 2" for the pair.
float AreaDyn = 1*2.5*3*(.0254*.0254);
float AreaFor = 1*2.5*(3+1.0/3.0)*(.0254*.0254); 
float Cd= .47, Cdr=1.15, Cdl=Cdr, Cdyn=Cdl, Cfor=Cdyn, Cdfus=1.00; // Cd=0.01, I set to Cd=Cdfus= .47 to get reasonable terminal velocity for semicircle nose rocket.
// */

float x = 3000*3*0 ;    // North Displacement
float dx = 0;
float ddx = 0;
float y = -4000*1*1/.3048;  // East Displacement  /.3048    Puts displacement in [m], In my program, distance compared to the SEE is actually in feet! WEIRD!!!
float angOff= 122.50;
float dy = 0;
float ddy = 0;
float z = 0.0; // [m]    // Altitude. "Up"
float dz = 0;  //
float ddz = 0;
float ub, vb, wb;
float dub, dvb, dwb;
float dxw, dyw, dzw;
// Target Location
float tx=0;  
float ty=0;
float tz=0;
float dtx=0, dty=0, dtz=0, ddtz= 0,  dddtz=0; // Target velocity.

// Rotational State Variables.
float alpha=0, dalpha=0, ddalpha=0;
float theta=0, dtheta=0, ddtheta=0;
float dalphaMAX=0, dthetaMAX=0, dpsiMAX=0; 
float ubMAX=0, vbMAX=0, wbMAX=0;
float psi=0, dpsi=0, ddpsi=0;
float ver; // 1-cos(angle);

// Set the initial attitude of Msl wrt target.
float alphaI=0*PI/180, dalphaI=0, ddalphaI=0; 
float thetaI= -atan2(z,sqrt(pow(x,2)+pow(y,2)))+ 1.0*PI/180.0/**/, dthetaI=0, ddthetaI=0;
float psiI= PI/2 +atan2(x,y)+ 1*PI/180, dpsiI=0*PI/180, ddpsiI=0;
// Set the target location w.r.t. being offset of the Msl nose.


// Max Flap Deflection
float maxFlapDeflection = 20.0; //  degrees is best.

// CONTROLLER INFORMATION
// Lyapunov Controller gains.
float Gp1=7, Gp2=Gp1, Gp3=Gp2;
float Gr1=7, Gr2=Gr1, Gr3=Gr2;  // Gr*=7 for pure torque control, = 1 good for LOD fin deflection.
float GFFy=0, GFFz=GFFy; 
float Kp1= 7, Kp2= Kp1, Kp3=Kp2; 
float Kr1=7, Kr2=Kr1, Kr3=Kr2;
float gamma=20; // gamma= 20
float cntrlx=0;  
float cntrly=0;
float cntrlz=0;
float cntrlxOld=0, cntrlyOld=0, cntrlzOld=0;
float cntrlyLOS=0, cntrlzLOS=0;
float cntrlSum;
float uslY=0, uslZ=0;
float cPercx=0, cPercy=0, cPercz=0;
float qx, qy, qz, b0x, b0y, b0z; 
float ix=1, jy=0, kz=0; 
float q0x=0;
float cntrlRoll=0;

ControlP5 cp5;
PeasyCam camera; 

// Object Initialization Below.
Pendulum mPendulum;  
ACGrad ACGrad;
ACMomentArms ACMomentArms;
ACDragReactForce ACDragReactForce;
Autopilot Autopilot; 

// */


OscP5 oscP5;
NetAddress mOscOut;

float posX=0, posY=0, posZ=0, rotX=0, rotY=0, rotZ=0, refAlt=0; 
float finplc1=0, finplc2=0, finplc3=0;
float finplc4=0;
boolean ctlPressed = false;
float g1i, g1j, g1k;
float xV, yV, zV; //
float normVelocity;





// 3-Loop Autopilot Initializations
 float E=0., ED=0., EDD=0.0;
 float Er=0., EDr=0., EDDr=0.0; 
 float DELq=0.0, DELr=0.0 ;
 float XKDC=1.0;
 float XK1=1,XK3=1;
 float VM, VM2;
 float CNA, CND, ZA,  ZD,  CMAP,CMA, CMD, XMA, XMD;
 float  Bal=.1, Q;
 float SREF=1, SWING, SfTAIL, STAIL, SPLAN;
 float TMP1, TMP2,TMP3,TMP4;
 float ALFTR;
 float WAF, ZAF;
 float THD;
 
 // PNG 
 float Rtmx=tx - x, Rtmy=ty-y, Rtmz=tz-z;
 float Vtmx= dtx-dx, Vtmy=dty-dy, Vtmz= dtz-dz;  // Inertial frame.
 float Vc1, Vc2, Vc3; // {Downrange, Crossrange}, "Vc"
 float XLAMD1, XLAMD1r, XLAMD1m1=0.0, XLAMD1rm1=0.0, dXLAMD1,dXLAMD1r,XLAMD1scd=0, XLAMD1rscd=0, dXLAMD1scd, dXLAMD1rscd; // Note: make first dXLAMD1 value calculated to be zero until the m1 variables are populated.
 float XNP=5;
 float aoa;
 float dgamma;  // Actual flight path angle.
 
 // More stuff
PVector[] LOSi, LOSiImp;  // Pursuer-Target LOS.
PVector[] uLOSi;
float Rgo1, Rgo2, Rgo3;
float RangeLOSNorm1;
float RangeLOSNorm1m1;
float dRangeLOSNorm1;
float Rub; 

// Linearized approximation of LOS  1-D zone
float YDD=0,YD=0,Y=0;

// Pure torque PNG
float XNC; // pure nc command
float thetadCMDkm1= 0;
float thetadCMDk = 0;
float XNC2; // pure nc command
float psidCMDkm1= 0;
float psidCMDk = 0;

float tgo=1000.0; // IC
      
PVector[] euquat, eurwquat, eumrquat;
boolean ckey4 = false;
float switch3 = 0.0;

float XNCLODMAX=0.0, XNCLODrMAX=0.0;
float switchLOD= 1;

float nez=0, angleFOV=0;
float tokDriftLim = 10.00; // usually i use 10g

float XLAMD0 = 0.0, XLAMDF, lambImp=0 , XLAMDoffset = 0, XLAMDTurn =0; // lambTurn: is the inertial frame "XLAMD" .

float ckeyImp = 1.0;
// Dust Trail
import java.util.*;
LinkedList<PointXY> trail;
int maxTrailLength = 400; // */

void setup() {
  // Choose a managable window size to plot simulation.
  // size(768, 400, P3D); // Form 3D window.
  // size(1024, 768, P3D); // Form 3D window.  // Best
  size(1068, 568, P3D); // Form 3D window. ERASE trying to make simulation act faster to print out object velocities.
  
  Grad = new PVector[3];  
  GradLado = new PVector[3];
  Gradr = new PVector[3];   
  Gradl = new PVector[3];   
  Gradn = new PVector[3];    
  Gradd1 = new PVector[3]; 
  Gradd2 = new PVector[3]; 
  Gradd3 = new PVector[3]; 
  Gradd4 = new PVector[3]; 

  Rwgrad= new PVector[3];
  RwgradLado= new PVector[3];
  Rwgradr= new PVector[3];
  Rwgradl= new PVector[3];
  Rwgradn= new PVector[3];
  Rwgrad1= new PVector[3];
  Rwgrad2= new PVector[3];
  Rwgrad3= new PVector[3];
  Rwgrad4= new PVector[3];
  Rwgradrf= new PVector[3];
  Rwgradlf= new PVector[3];
  PosVec= new PVector[3]; 


  rdf1=new PVector[3]; 
  rdf2=new PVector[3]; 
  rdf3=new PVector[3]; 
  rdf4=new PVector[3]; 
  rdf1d=new PVector[3]; 
  rdf2d=new PVector[3]; 
  rdf3d=new PVector[3]; 
  rdf4d=new PVector[3]; 
  rdf1f=new PVector[3]; 
  rdf2f=new PVector[3]; 
  rdf3f=new PVector[3]; 
  rdf4f=new PVector[3];   
  Mf1= new PVector[3]; 
  Mf2= new PVector[3]; 
  Mf3= new PVector[3]; 
  Mf4= new PVector[3];
  Mff1= new PVector[3]; 
  Mff2= new PVector[3]; 
  Mff3= new PVector[3]; 
  Mff4= new PVector[3];  
  Mfd1= new PVector[3]; 
  Mfd2= new PVector[3]; 
  Mfd3= new PVector[3]; 
  Mfd4= new PVector[3];
  MfIN=  new PVector[3]; 

  th= new PVector[3];
  ZEM= new PVector[3];
  ZEMparr= new PVector[3];
  ZEMperp= new PVector[3];

  LOSi = new PVector[3];
  LOSiImp = new PVector[3];
  
  uLOSi = new PVector[3];
  euquat = new PVector[3];
  eurwquat = new PVector[3];
  eumrquat = new PVector[3];

  Grad[1]= new PVector(0, 0, 0);
  GradLado[1] = new PVector(0, 0, 0);
  Gradr[1] = new PVector(0, 0, 0);
  Gradl[1] = new PVector(0, 0, 0);
  Gradn[1] = new PVector(0, 0, 0);
  Gradd1[1] = new PVector(0, 0, 0);
  Gradd2[1] = new PVector(0, 0, 0);
  Gradd3[1] = new PVector(0, 0, 0);
  Gradd4[1] = new PVector(0, 0, 0);
  Rwgrad[1]= new PVector(1, 1, 1); // 
  RwgradLado[1]= new PVector(1, 1, 1);
  Rwgradr[1]= new PVector(1, 1, 1); //
  Rwgradl[1]= new PVector(1, 1, 1); 
  Rwgradn[1]= new PVector(1, 1, 1); 
  Rwgrad1[1]= new PVector(1, 1, 1);
  Rwgrad2[1]= new PVector(1, 1, 1);
  Rwgrad3[1]= new PVector(1, 1, 1);
  Rwgrad4[1]= new PVector(1, 1, 1);
  Rwgradrf[1]= new PVector(1, 1, 1);
  Rwgradlf[1]= new PVector(1, 1, 1);

  PosVec[1]= new PVector(1, .2, -.5);

  

  rdf1[1]= new PVector(0, 0, 0);
  rdf2[1]= new PVector(0, 0, 0);
  rdf3[1]= new PVector(0, 0, 0); 
  rdf4[1]= new PVector(0, 0, 0);
  rdf1d[1]= new PVector(0, 0, 0);
  rdf2d[1]= new PVector(0, 0, 0);
  rdf3d[1]= new PVector(0, 0, 0); 
  rdf4d[1]= new PVector(0, 0, 0);
  rdf1f[1]= new PVector(0, 0, 0);
  rdf2f[1]= new PVector(0, 0, 0);
  rdf3f[1]= new PVector(0, 0, 0); 
  rdf4f[1]= new PVector(0, 0, 0);
  Mf1[1]= new PVector(0, 0, 0);
  Mf2[1]= new PVector(0, 0, 0);
  Mf3[1]= new PVector(0, 0, 0);
  Mf4[1]= new PVector(0, 0, 0);
  Mff1[1]= new PVector(0, 0, 0);
  Mff2[1]= new PVector(0, 0, 0);
  Mff3[1]= new PVector(0, 0, 0);
  Mff4[1]= new PVector(0, 0, 0);  
  Mfd1[1]= new PVector(0, 0, 0);
  Mfd2[1]= new PVector(0, 0, 0);
  Mfd3[1]= new PVector(0, 0, 0);
  Mfd4[1]= new PVector(0, 0, 0);
  MfIN[1]= new PVector(0, 0, 0);

  th[1]= new PVector(0, 0, 0);
  
  ZEM[1]= new PVector(0, 0, 0);
  ZEMparr[1]= new PVector(0, 0, 0);
  ZEMperp[1]= new PVector(0, 0, 0);
  
  // Below is initial values.
  LOSi[1] = new PVector(0, 0, 0);
  LOSiImp[1] = new PVector(0, 0, 0); // Now I can set LOSiImp values.
  LOSi[1].x = -(x - tx) ; LOSi[1].y = -(y - ty); LOSi[1].z = -(z - tz);
  uLOSi[1] = new PVector(0, 0, 0);
  uLOSi[1].x = LOSi[1].x/sqrt(pow(LOSi[1].x,2)+pow(LOSi[1].y,2)) ; uLOSi[1].y = LOSi[1].y/sqrt(pow(LOSi[1].x,2)+pow(LOSi[1].y,2));
  uLOSi[1].z = LOSi[1].z/sqrt(pow(LOSi[1].z,2)+pow(LOSi[1].z,2));
  
  euquat[1] = new PVector(0,0,0);
  eurwquat[1] = new PVector(0,0,0);
  eumrquat[1] = new PVector(0,0,0);
  
  smooth();
  frameRate((int)(1/(dt1*1)));
  //frameRate(200); // 900 a good value.
  //frameRate(900); // 900 a good value.
  textSize(20); 

  mOscOut = new NetAddress("192.168.0.24", 8888);

  camera = new PeasyCam(this, 650); // 

  camera.setRotations(-1.0, 0.0, 0.0); // 
  camera.lookAt(8.0, -50.0, 80.0); //  

  mPendulum = new Pendulum(1); // Create the class object in the platform file.  Note: s=1.
  mPendulum.applyTranslationAndRotation(new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(),new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector());  /* Run private class function in the object to command object*/

  ACGrad = new ACGrad(1);
  ACGrad.applyTranslationAndRotation(new PVector(), new PVector(), new PVector());  /* Run private class function in the object to command object*/

  ACMomentArms = new ACMomentArms(1);
  ACMomentArms.applyTranslationAndRotation(new PVector());  /* Run private class function in the object to command object*/

  ACDragReactForce = new ACDragReactForce(1);
  ACDragReactForce.applyTranslationAndRotation(new PVector(),new PVector(),new PVector(),new PVector(),new PVector(),new PVector(),new PVector(),new PVector(),new PVector(),new PVector(),new PVector()); 

  Autopilot = new Autopilot(1);
  Autopilot.applyTranslationAndRotation(new PVector(),new PVector(),new PVector(),new PVector(),new PVector());  /* Run private class function in the object to command object*/

  // Initialize A/C Orientation for simulation, and reference vector for control.
  ix=1; 
  jy=0; 
  kz=0;  
  qx=1; 
  qy=0; 
  qz=0;  
  q0x=0;
  b0x=1;
  b0y=0;
  b0z=0;

  cp5 = new ControlP5(this); // Create control object.
  cp5.addSlider("ix").setPosition(width-210, 20).setSize(180, 40).setRange(-1, 1);
  cp5.addSlider("jy").setPosition(width-210, 70).setSize(180, 40).setRange(-1, 1);
  cp5.addSlider("kz").setPosition(width-210, 120).setSize(180, 40).setRange(-1, 1);// */  
  //cp5.addSlider("cntrlRoll").setPosition(width-210, 170).setSize(180, 40).setRange(-150, 150);  
  cp5.addSlider("Rub").setPosition(width-210, 170).setSize(180, 40).setRange(0, 300);  
  cp5.addSlider("XLAMDoffset").setPosition(width-210, 220).setSize(180, 40).setRange(0.0* PI/180.0 , angOff* PI/180.0); // Delta impact angle off of initially detected only!. 
  
  
  cp5.setAutoDraw(false); 
  camera.setActive(true); 
  // Dynamics Simulation.
  llave= false; // For triggering the dynamic laws on the system.
  
  trail = new LinkedList<PointXY>();
  stroke(249, 233, 7);
  strokeWeight(1);
  
}

//Dust/smoke Trail, function
public class PointXY {
  public float x, y, z;
  public PointXY(float px, float py, float pz) {
    x = px;
    y = py;
    z = pz;
  }
}

void draw() {  //
  background(0); 

  // Populate Smoke trail of the A/C
  if (trail.size() >= 2) {
    PointXY currPoint, lastPoint = trail.get(0);
    for (int i = 0; i < trail.size(); i++) {
      currPoint = trail.get(i);
      //stroke(249, 233, 7);
      strokeWeight(1);
      stroke(255);
      line(lastPoint.x, lastPoint.y, lastPoint.z, currPoint.x, currPoint.y, currPoint.z);
      lastPoint = currPoint;
    }
  }
  PointXY p = new PointXY(x, y, z); // call in values into the new linked list, can scope here see these variables?
  trail.addFirst(p);
  
  
  // Below is ACGrad update!
  ACGrad.applyTranslationAndRotation(PVector.mult(new PVector(rotX, rotY, rotZ), MAX_ROTATION), 
  PVector.mult(new PVector(finplc1, finplc2, finplc3), 1), 
  PVector.mult(new PVector(finplc4, 0, 0), 1)); // */
  
  PVector Gradm = ACGrad.getGrad();
  PVector GradLadom = ACGrad.getGradLado(); 
  PVector Gradrm = ACGrad.getGradr() ;
  PVector Gradlm = ACGrad.getGradl();
  PVector Gradnm = ACGrad.getGradn() ;
  PVector Gradd1m = ACGrad.getGradd1();
  PVector Gradd2m = ACGrad.getGradd2();
  PVector Gradd3m = ACGrad.getGradd3();
  PVector Gradd4m = ACGrad.getGradd4();

  Grad[1].x= Gradm.x;
  Grad[1].y= Gradm.y;
  Grad[1].z= Gradm.z;
  GradLado[1].x= GradLadom.x;
  GradLado[1].y= GradLadom.y;
  GradLado[1].z= GradLadom.z;
  Gradr[1].x=Gradrm.x;
  Gradr[1].y=Gradrm.y;
  Gradr[1].z=Gradrm.z;
  Gradl[1].x=Gradlm.x;
  Gradl[1].y=Gradlm.y;
  Gradl[1].z=Gradlm.z;
  Gradn[1].x= Gradnm.x;
  Gradn[1].y= Gradnm.y;
  Gradn[1].z= Gradnm.z;
  Gradd1[1].x= Gradd1m.x;
  Gradd1[1].y= Gradd1m.y;
  Gradd1[1].z= Gradd1m.z;
  Gradd3[1].x= Gradd3m.x;
  Gradd3[1].y= Gradd3m.y;
  Gradd3[1].z= Gradd3m.z;
  Gradd2[1].x= Gradd2m.x;
  Gradd2[1].y= Gradd2m.y;
  Gradd2[1].z= Gradd2m.z;
  Gradd4[1].x= Gradd4m.x;
  Gradd4[1].y= Gradd4m.y;
  Gradd4[1].z= Gradd4m.z;
  
  
  // Pass to PENDULUM CLASS OBJECT NOW HERE.
  mPendulum.applyTranslationAndRotation(PVector.mult(new PVector(posX, posY, posZ), MAX_TRANSLATION), 
    PVector.mult(new PVector(rotX, rotY, rotZ), MAX_ROTATION), 
    PVector.mult(new PVector(Grad[1].x, Grad[1].y, Grad[1].z), 1), // To See the gradient vector.
    PVector.mult(new PVector(ix, jy, kz), 1), 
    PVector.mult(new PVector(finplc1, finplc2, finplc3), 1), 
    PVector.mult(new PVector(finplc4, 0.0, guidFlagInt), 1),
    PVector.mult(new PVector(Gradr[1].x, Gradr[1].y, Gradr[1].z), 1), 
    PVector.mult(new PVector(Gradl[1].x, Gradl[1].y, Gradl[1].z), 1), 
    PVector.mult(new PVector(Gradn[1].x, Gradn[1].y, Gradn[1].z), 1), 
    PVector.mult(new PVector(rdf1[1].x/inToMeters, rdf1[1].y/inToMeters, rdf1[1].z/inToMeters), 1), 
    PVector.mult(new PVector(rdf2[1].x/inToMeters, rdf2[1].y/inToMeters, rdf2[1].z/inToMeters), 1), 
    PVector.mult(new PVector(rdf3[1].x/inToMeters, rdf3[1].y/inToMeters, rdf3[1].z/inToMeters), 1), 
    PVector.mult(new PVector(rdf4[1].x/inToMeters, rdf4[1].y/inToMeters, rdf4[1].z/inToMeters), 1), 
    PVector.mult(new PVector(Gradd1[1].x, Gradd1[1].y, Gradd1[1].z), 1), 
    PVector.mult(new PVector(Gradd2[1].x, Gradd2[1].y, Gradd2[1].z), 1), 
    PVector.mult(new PVector(Gradd3[1].x, Gradd3[1].y, Gradd3[1].z), 1), 
    PVector.mult(new PVector(Gradd4[1].x, Gradd4[1].y, Gradd4[1].z), 1), 
    PVector.mult(new PVector(th[1].x, th[1].y, th[1].z), 1), 
    PVector.mult(new PVector(Rwgrad1[1].x, Rwgrad1[1].y, Rwgrad1[1].z), 1), 
    PVector.mult(new PVector(Rwgrad2[1].x, Rwgrad2[1].y, Rwgrad2[1].z), 1), 
    PVector.mult(new PVector(Rwgrad3[1].x, Rwgrad3[1].y, Rwgrad3[1].z), 1), 
    PVector.mult(new PVector(Rwgrad4[1].x, Rwgrad4[1].y, Rwgrad4[1].z), 1), 
    PVector.mult(new PVector(tx, ty, tz), 1)); // Print he moment arm last. To verify the moment math will be correct.

  PVector euquaTemp = mPendulum.getEuQuat();
  PVector eurwquaTemp=  mPendulum.getEurwquat();
  PVector eumrquaTemp=  mPendulum.getEumrquat();
  //
  euquat[1].x= euquaTemp.x;
  euquat[1].y= euquaTemp.y;
  euquat[1].z= euquaTemp.z;
  
  eurwquat[1].x= eurwquaTemp.x;
  eurwquat[1].y= eurwquaTemp.y;
  eurwquat[1].z= eurwquaTemp.z;
  
  eumrquat[1].x= eumrquaTemp.x;
  eumrquat[1].y= eumrquaTemp.y;
  eumrquat[1].z= eumrquaTemp.z;
  
  //Gradn[1].*
  //eurwquat[1].*
  //eumrquat[1].*

  // Below is moment arm update
  ACMomentArms.applyTranslationAndRotation(PVector.mult(new PVector(rotX, rotY, rotZ), MAX_ROTATION)); // */
  
  PVector rdf1m = ACMomentArms.getRdf1() ;
  PVector rdf1dm  = ACMomentArms.getRdf1d();
  PVector rdf1fm  = ACMomentArms.getRdf1f();
  PVector rdf2m   = ACMomentArms.getRdf2() ;
  PVector rdf2dm  = ACMomentArms.getRdf2d();
  PVector rdf2fm = ACMomentArms.getRdf2f();
  PVector rdf3m   = ACMomentArms.getRdf3() ;
  PVector rdf3dm  = ACMomentArms.getRdf3d();
  PVector rdf3fm = ACMomentArms.getRdf3f();
  PVector rdf4m   = ACMomentArms.getRdf4() ;
  PVector rdf4dm  = ACMomentArms.getRdf4d();
  PVector rdf4fm = ACMomentArms.getRdf4f();

  rdf1[1].x=  rdf1m.x;
  rdf1[1].y=  rdf1m.y;
  rdf1[1].z=  rdf1m.z;
  rdf1d[1].x= rdf1dm.x;
  rdf1d[1].y= rdf1dm.y;
  rdf1d[1].z= rdf1dm.z;
  rdf1f[1].x= rdf1fm.x;
  rdf1f[1].y= rdf1fm.y;
  rdf1f[1].z= rdf1fm.z;

  rdf2[1].x=  rdf2m.x;
  rdf2[1].y=  rdf2m.y;
  rdf2[1].z=  rdf2m.z;
  rdf2d[1].x= rdf2dm.x;
  rdf2d[1].y= rdf2dm.y;
  rdf2d[1].z= rdf2dm.z;
  rdf2f[1].x= rdf2fm.x;
  rdf2f[1].y= rdf2fm.y;
  rdf2f[1].z= rdf2fm.z;

  rdf3[1].x=  rdf3m.x;  
  rdf3[1].y=  rdf3m.y;
  rdf3[1].z=  rdf3m.z;
  rdf3d[1].x= rdf3dm.x;
  rdf3d[1].y= rdf3dm.y;
  rdf3d[1].z= rdf3dm.z;
  rdf3f[1].x= rdf3fm.x;
  rdf3f[1].y= rdf3fm.y;
  rdf3f[1].z= rdf3fm.z;

  rdf4[1].x=  rdf4m.x;  
  rdf4[1].y=  rdf4m.y;
  rdf4[1].z=  rdf4m.z;
  rdf4d[1].x= rdf4dm.x;
  rdf4d[1].y= rdf4dm.y;
  rdf4d[1].z= rdf4dm.z;
  rdf4f[1].x= rdf4fm.x;
  rdf4f[1].y= rdf4fm.y;
  rdf4f[1].z= rdf4fm.z;

  // LOS Designation
  PosVec[1].x= tx-x;
  PosVec[1].y= ty-y;
  PosVec[1].z= tz-z;
  PosVecNorm= sqrt(PosVec[1].x*PosVec[1].x + PosVec[1].y*PosVec[1].y+ PosVec[1].z*PosVec[1].z);
  // LOS
   ix =PosVec[1].x/PosVecNorm;
   jy =PosVec[1].y/PosVecNorm;
   kz =PosVec[1].z/PosVecNorm; // */   
  // Get LOS unit vector

  // Retrieve targetig information for implementing controllers.
  PVector erqrefqk = mPendulum.getErQuatError();  
  PVector TargQuat = mPendulum.getTargQuat();  // 
 
  b0x=TargQuat.x;
  b0y=TargQuat.y;
  b0z=TargQuat.z;

  // Note Placing the text here doesn't crash the sim, weird!
  if (z==0) {
    textSize(15);
    fill(255, 10, 10);
    text("MissRadius [x;y]", 75, 60-100, 10); 
    textSize(20);
    fill(0, 102, 153);
    text(String.format("%.2f", x-tx), 75, 80-100, 10);
    text(String.format("%.2f", y-ty), 75, 100-100, 10);
  }

  textSize(15);
  fill(0, 255, 255);
  text("dz;z", 75, 60, 10); 
  textSize(20);
  fill(0, 102, 153);
  text(String.format("%.2f", dz), 75, 80, 10);
  text(String.format("%.2f", z), 75, 100, 10);

  angleFOV = acos(uLOSi[1].x*euquat[1].x   + uLOSi[1].y*euquat[1].y   + uLOSi[1].z*euquat[1].z);
  if (angleFOV< 1.0*PI/180.0){
    switchLOD = 1; // Use fin png, 
  }   // */

  textSize(10);
  fill(0, 255, 255);
  text("_DC", 75, 130, 10); 
  textSize(20);
  fill(0, 102, 153);
  text(String.format("%.2f", tgo), 75, 150, 10);
  text(String.format("%.2f", switchLOD ), 75, 190, 10);

  /*text(String.format("%.2f",switchLOD ), 75, 190, 10);
  text(String.format("%.2f",angleFOV*180/PI ), 75, 210, 10); // */

  textSize(15);
  fill(0, 255, 255);
  text("dy;y", 200, 60, 10); 
  textSize(20);
  fill(0, 102, 153);
  text(String.format("%.2f", dy), 200, 80, 10);
  text(String.format("%.2f", y), 200, 100, 10);

  textSize(15);
  fill(0, 255, 255);
  text("dx;x", 325, 60, 10); 
  textSize(20);
  fill(0, 102, 153);
  text(String.format("%.2f", dx), 325, 80, 10);
  text(String.format("%.2f", x), 325, 100, 10);

  textSize(10);
  fill(0, 255, 255);
  text("time[secs]", 325, 190-60, 10); 
  textSize(20);
  fill(0, 102, 153);
  text(String.format("%.2f", tsec), 325, 210-60, 10);


// Record maximum angular rates of the A/C
  textSize(10);
  fill(0, 255, 255);
  text("p,q,r MAX", -50, 140-80, 10); 
  textSize(20);
  fill(0, 102, 153);
  text(String.format("%.2f", (dalphaMAX*180/PI) ), -50, 160-80, 10);
  text(String.format("%.2f", (dthetaMAX*180/PI) ), -50, 180-80, 10);
  text(String.format("%.2f", (dpsiMAX*180/PI)   ), -50, 200-80, 10); // */
  
  // Record maximum body frame rates of the A/C
  textSize(10);
  fill(0, 255, 255);
  text("ub,vb,wb MAX", -50, 240-80, 10); 
  textSize(20);
  fill(0, 102, 153);
  text(String.format("%.2f", (ubMAX) ), -50, 260-80, 10);
  text(String.format("%.2f", (vbMAX) ), -50, 280-80, 10);
  text(String.format("%.2f", (wbMAX)   ), -50, 300-80, 10);
  text(String.format("%.2f", (switchLOD)   ), -50, 320-80, 10);// */
  

  
  
  textSize(10);
  fill(0, 255, 255);
  text("Vel_body", -150, 140-80, 10); 
  textSize(20);
  fill(0, 102, 153);
  text(String.format("%.2f", (ub) ), -150, 160-80, 10);
  text(String.format("%.2f", (vb) ), -150, 180-80, 10);
  text(String.format("%.2f", (wb) ), -150, 200-80, 10);
  
  mPendulum.draw(); 

  hint(DISABLE_DEPTH_TEST); 
  camera.beginHUD(); //?
  cp5.draw();
  camera.endHUD();
  hint(ENABLE_DEPTH_TEST); // Play with this.

  // BELOW TURNS SIMUNATION ENVIRONMENT ON IF KEYS AND SWITCHED.
  if (llave) {  // Apply dynamic laws on the system. Activate Dynamics.
    // EOM OF POSITION ( In Inertial frame.)
    posX = x;
    posY = y;
    posZ = z;
    tsec +=dt1; //
    rotX = alphaI;
    rotY = thetaI;
    rotZ = psiI;
    if (tsec>ballistTime) {
      guidFlag=true;   // guidance system on.   
      guidFlagInt=1;
    }
    
    dty = -0;
    if (z<=0) {
     dty=0; 
    }
    ty = ty + dty*dt1;
    
    //ddtz =  200*PI/2-pow((PI/2),2)*tz;  // Laplace domain, harder to derive.
    //dddtz = -200*pow(PI/2,2)*sin(PI/2*tsec);
    //ddtz = 200*PI/2*cos(PI/2*tsec);
    //dddtz = -1;  // step jerk.
   // ddtz = ddtz + dddtz*dt1;
    //dtz = dtz + ddtz*dt1;
    // Freeze if close.
   /* if (PosVecNorm<=20) { 
      dtz=0;
    } // */
    //tz = tz + dtz*dt1;
    
    // PNG Logic
    LOSi[1].x = -(x - tx) ; LOSi[1].y = -(y - ty); LOSi[1].z = -(z - tz); // Line of Sight
    RangeLOSNorm1m1= RangeLOSNorm1;
    RangeLOSNorm1= sqrt(pow(LOSi[1].x,2)+pow(LOSi[1].y,2)+pow(LOSi[1].z,2));
    uLOSi[1].x = LOSi[1].x/RangeLOSNorm1 ; uLOSi[1].y = LOSi[1].y/RangeLOSNorm1; uLOSi[1].z = LOSi[1].z/RangeLOSNorm1;
    
    Rgo1= sqrt(pow(LOSi[1].x,2)+pow(LOSi[1].z,2));  
    Rgo2= sqrt(pow(LOSi[1].x,2)+pow(LOSi[1].y,2)); 
    Rgo3= sqrt(pow(LOSi[1].y,2)+pow(LOSi[1].z,2));
    Rtmx= tx - x;
    Rtmy= ty - y;
    Rtmz= tz - z;
    Vtmx = dtx-dx; Vtmy= dty - dy; Vtmz= dtz - dz;  
    Vc1 = -(LOSi[1].x*Vtmx+LOSi[1].z*Vtmz)/Rgo1; 
    Vc2 = -(LOSi[1].x*Vtmx+LOSi[1].y*Vtmy)/Rgo2;
    Vc3 = -(LOSi[1].y*Vtmy+LOSi[1].z*Vtmz)/Rgo3; 
    
    /*if(PosVecNorm < 500) {
      Vc1 = 100.0;
      Vc2 = 100.0;     
      Vc3 = 100.0;
    } // */
    
// Bodyframe detection of LOS
  nez =  LOSi[1].x*euquat[1].x   + LOSi[1].y*euquat[1].y   + LOSi[1].z*euquat[1].z;  // Dot product for projection of LOS onto body frame coordinates
  float mige = LOSi[1].x*eurwquat[1].x + LOSi[1].y*eurwquat[1].y + LOSi[1].z*eurwquat[1].z;
  float tete = LOSi[1].x*eumrquat[1].x + LOSi[1].y*eumrquat[1].y + LOSi[1].z*eumrquat[1].z;
   XLAMD1rm1 = XLAMD1r;

   // Note the LOS is not wrt seeker head to target, but msl position to target position. This is what failed the PNG in 2D
   
    XLAMD1m1  = XLAMD1;
    XLAMD1 = atan(PosVec[1].z/PosVec[1].x); 
    
     Rgo1 = sqrt(pow(LOSi[1].x,2)+pow(LOSi[1].z,2));  
     Rgo2 = sqrt(pow(LOSi[1].x,2)+pow(LOSi[1].y,2)); 
     Rgo3 = sqrt(pow(LOSi[1].y,2)+pow(LOSi[1].z,2));
    

    switch3 =0.0;
    if(psiI*180/PI> 45.0 && psiI*180/PI<135 )  {
      switch3 = 1.0;
    }
     if (psiI*180/PI<315 && psiI*180/PI>225) {
       switch3 = 1.0;
     } //*/
    
    dXLAMD1= (1-switch3)*x/abs(x)*((LOSi[1].x)*Vtmz - LOSi[1].z*Vtmx)/(pow(Rgo1,2)) + (switch3)*y/abs(y)*(LOSi[1].y*Vtmz - LOSi[1].z*Vtmy)/(pow(Rgo3,2))  ; // This works, the average of two controllers
      
    dXLAMD1r= (LOSi[1].x*Vtmy - LOSi[1].y*Vtmx)/(pow(Rgo2,2)); 
   
    

    
    dRangeLOSNorm1 =  (RangeLOSNorm1 - RangeLOSNorm1m1)/dt1; 
    
    // UNIT VELOCITY VECTOR in inertial frame calculated below.
    ub=cos(rotZ)*cos(rotY)                                *dx - sin(rotZ)*cos(rotY)                                  *dy + sin(rotY)          *dz ; 
    vb=(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*dx + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*dy - cos(rotY)*sin(rotX)*dz ;
    wb=(sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*dx + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*dy +cos(rotY)*cos(rotX) *dz;    
   

      // Boring msl, saturate body frame velocity and convert back into inertial frame!
      if (abs(vb) > tokDriftLim) {
        vb = vb/abs(vb)*tokDriftLim;
      }
      if (abs(wb) > tokDriftLim) {
        wb = wb/abs(wb)*tokDriftLim;
      }

    dx= cos(rotZ)*cos(rotY)   *ub + (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))  *vb + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))  *wb ; 
    dy= - sin(rotZ)*cos(rotY) *ub + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*vb + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*wb ;
    dz=   sin(rotY)           *ub - cos(rotY)*sin(rotX)                                  *vb +                                cos(rotY)*cos(rotX)   *wb ;   
    
    dub=cos(rotZ)*cos(rotY)                                *ddx - sin(rotZ)*cos(rotY)                                  *ddy + sin(rotY)          *ddz;
    dvb=(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*ddx + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*ddy - cos(rotY)*sin(rotX)*ddz;
    dwb=(sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*ddx + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*ddy +cos(rotY)*cos(rotX) *ddz;
    xV=dx; 
    yV=dy;
    zV=dz;

    normVelocity= sqrt(xV*xV+yV*yV+zV*zV) ;
    if (normVelocity !=0) {
      xV= xV/normVelocity;
      yV= yV/normVelocity;
      zV= zV/normVelocity;
    } 

  // Below is ACDragReactForce update!
    ACDragReactForce.applyTranslationAndRotation(PVector.mult(new PVector(normVelocity, 0, 0), 1), 
    PVector.mult(new PVector(xV, yV, zV), 1),
    PVector.mult(new PVector(Grad[1].x, Grad[1].y, Grad[1].z), 1),
    PVector.mult(new PVector(GradLado[1].x, GradLado[1].y, GradLado[1].z), 1),
    PVector.mult(new PVector(Gradn[1].x, Gradn[1].y, Gradn[1].z), 1),
    PVector.mult(new PVector(Gradr[1].x, Gradr[1].y, Gradr[1].z), 1),
    PVector.mult(new PVector(Gradl[1].x, Gradl[1].y, Gradl[1].z), 1),
    PVector.mult(new PVector(Gradd1[1].x, Gradd1[1].y, Gradd1[1].z), 1),
    PVector.mult(new PVector(Gradd2[1].x, Gradd2[1].y, Gradd2[1].z), 1),
    PVector.mult(new PVector(Gradd3[1].x, Gradd3[1].y, Gradd3[1].z), 1),
    PVector.mult(new PVector(Gradd4[1].x, Gradd4[1].y, Gradd4[1].z), 1) ); // */

    PVector Rwgradm = ACDragReactForce.getRwgrad() ;
    PVector RwgradLadom= ACDragReactForce.getRwgradLado();
    PVector Rwgradrm= ACDragReactForce.getRwgradr()   ;
    PVector Rwgradlm= ACDragReactForce.getRwgradl()   ;
    PVector Rwgradnm= ACDragReactForce.getRwgradn()   ;
    PVector Rwgrad1m= ACDragReactForce.getRwgrad1()   ;
    PVector Rwgrad2m= ACDragReactForce.getRwgrad2()   ;
    PVector Rwgrad3m= ACDragReactForce.getRwgrad3()   ;
    PVector Rwgrad4m= ACDragReactForce.getRwgrad4()   ;
    PVector Rwgradlfm= ACDragReactForce.getRwgradlf()  ;
    PVector Rwgradrfm= ACDragReactForce.getRwgradrf()  ;
    
    Rwgrad[1].x= Rwgradm.x;
    Rwgrad[1].y= Rwgradm.y;
    Rwgrad[1].z= Rwgradm.z;

    RwgradLado[1].x=RwgradLadom.x;
    RwgradLado[1].y=RwgradLadom.y;
    RwgradLado[1].z=RwgradLadom.z;
    
    Rwgradr[1].x=Rwgradrm.x;
    Rwgradr[1].y=Rwgradrm.y;
    Rwgradr[1].z=Rwgradrm.z;

    Rwgradl[1].x=Rwgradlm.x;
    Rwgradl[1].y=Rwgradlm.y;
    Rwgradl[1].z=Rwgradlm.z;

    Rwgrad1[1].x=Rwgrad1m.x;
    Rwgrad1[1].y=Rwgrad1m.y;
    Rwgrad1[1].z=Rwgrad1m.z;
 
    Rwgradlf[1].x=Rwgradlfm.x;
    Rwgradlf[1].y=Rwgradlfm.y;
    Rwgradlf[1].z=Rwgradlfm.z;

    Rwgrad3[1].x=Rwgrad3m.x;
    Rwgrad3[1].y=Rwgrad3m.y;
    Rwgrad3[1].z=Rwgrad3m.z;

    Rwgrad2[1].x=Rwgrad2m.x;
    Rwgrad2[1].y=Rwgrad2m.y;
    Rwgrad2[1].z=Rwgrad2m.z;
   
    Rwgradrf[1].x=Rwgradrfm.x;
    Rwgradrf[1].y=Rwgradrfm.y;
    Rwgradrf[1].z=Rwgradrfm.z;

    Rwgrad4[1].x= Rwgrad4m.x;
    Rwgrad4[1].y= Rwgrad4m.y;
    Rwgrad4[1].z= Rwgrad4m.z;

    Rwgradn[1].x= Rwgradnm.x;
    Rwgradn[1].y= Rwgradnm.y;
    Rwgradn[1].z= Rwgradnm.z;
    
    // Pursuant thrust
    if (tsec<burnTime) {
     thrMagn=7450.77; // [N]
     } else {
     thrMagn=Rub;
     } // */
     
    // Thrust EOM below.
    th[1].x=Gradn[1].x*thrMagn; 
    th[1].y=Gradn[1].y*thrMagn; 
    th[1].z=Gradn[1].z*thrMagn; 


// Start "Moment" computation "function" section

    Mf1[1].x= 0        *Rwgradl[1].x - rdf1[1].z*Rwgradl[1].y + rdf1[1].y*Rwgradl[1].z; 
    Mf1[1].y= rdf1[1].z*Rwgradl[1].x + 0        *Rwgradl[1].y - rdf1[1].x*Rwgradl[1].z; 
    Mf1[1].z=-rdf1[1].y*Rwgradl[1].x + rdf1[1].x*Rwgradl[1].y + 0        *Rwgradl[1].z; 
    Mf1[1].x= Mf1[1].x/2.0;
    Mf1[1].y= Mf1[1].y/2.0;
    Mf1[1].z= Mf1[1].z/2.0;
    
    MfIN[1].x= Mf1[1].x;
    MfIN[1].y= Mf1[1].y;
    MfIN[1].z= Mf1[1].z;
    Mf1[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mf1[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mf1[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mfd1[1].x= 0        *Rwgrad1[1].x - rdf1d[1].z*Rwgrad1[1].y + rdf1d[1].y*Rwgrad1[1].z; 
    Mfd1[1].y= rdf1d[1].z*Rwgrad1[1].x + 0        *Rwgrad1[1].y - rdf1d[1].x*Rwgrad1[1].z; 
    Mfd1[1].z=-rdf1d[1].y*Rwgrad1[1].x + rdf1d[1].x*Rwgrad1[1].y + 0        *Rwgrad1[1].z; 
    MfIN[1].x= Mfd1[1].x;
    MfIN[1].y= Mfd1[1].y;
    MfIN[1].z= Mfd1[1].z;
    Mfd1[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mfd1[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mfd1[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mff1[1].x= 0         *Rwgradlf[1].x - rdf1f[1].z*Rwgradlf[1].y + rdf1f[1].y*Rwgradlf[1].z; 
    Mff1[1].y= rdf1f[1].z*Rwgradlf[1].x + 0         *Rwgradlf[1].y - rdf1f[1].x*Rwgradlf[1].z; 
    Mff1[1].z=-rdf1f[1].y*Rwgradlf[1].x + rdf1f[1].x*Rwgradlf[1].y + 0         *Rwgradlf[1].z; 

    MfIN[1].x= Mff1[1].x;
    MfIN[1].y= Mff1[1].y;
    MfIN[1].z= Mff1[1].z;
    Mff1[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mff1[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mff1[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mf2[1].x= 0        *Rwgradr[1].x - rdf2[1].z*Rwgradr[1].y + rdf2[1].y*Rwgradr[1].z; 
    Mf2[1].y= rdf2[1].z*Rwgradr[1].x + 0        *Rwgradr[1].y - rdf2[1].x*Rwgradr[1].z; 
    Mf2[1].z= -rdf2[1].y*Rwgradr[1].x + rdf2[1].x*Rwgradr[1].y + 0       *Rwgradr[1].z; 
    Mf2[1].x= Mf2[1].x/2.0;
    Mf2[1].y= Mf2[1].y/2.0;
    Mf2[1].z= Mf2[1].z/2.0;
    
    MfIN[1].x= Mf2[1].x;
    MfIN[1].y= Mf2[1].y;
    MfIN[1].z= Mf2[1].z;
    Mf2[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mf2[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mf2[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mfd2[1].x= 0        *Rwgrad2[1].x - rdf2d[1].z*Rwgrad2[1].y + rdf2d[1].y*Rwgrad2[1].z; 
    Mfd2[1].y= rdf2d[1].z*Rwgrad2[1].x + 0        *Rwgrad2[1].y - rdf2d[1].x*Rwgrad2[1].z; 
    Mfd2[1].z=-rdf2d[1].y*Rwgrad2[1].x + rdf2d[1].x*Rwgrad2[1].y + 0        *Rwgrad2[1].z; 
    MfIN[1].x= Mfd2[1].x;
    MfIN[1].y= Mfd2[1].y;
    MfIN[1].z= Mfd2[1].z;
    Mfd2[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mfd2[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mfd2[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mff2[1].x= 0         *Rwgradrf[1].x - rdf2f[1].z*Rwgradrf[1].y + rdf2f[1].y*Rwgradrf[1].z; 
    Mff2[1].y= rdf2f[1].z*Rwgradrf[1].x +  0        *Rwgradrf[1].y - rdf2f[1].x*Rwgradrf[1].z; 
    Mff2[1].z=-rdf2f[1].y*Rwgradrf[1].x + rdf2f[1].x*Rwgradrf[1].y + 0         *Rwgradrf[1].z; 

    MfIN[1].x= Mff2[1].x;
    MfIN[1].y= Mff2[1].y;
    MfIN[1].z= Mff2[1].z;
    Mff2[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mff2[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mff2[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mf3[1].x= 0        *Rwgradl[1].x - rdf3[1].z*Rwgradl[1].y + rdf3[1].y*Rwgradl[1].z; 
    Mf3[1].y= rdf3[1].z*Rwgradl[1].x + 0        *Rwgradl[1].y - rdf3[1].x*Rwgradl[1].z; 
    Mf3[1].z= -rdf3[1].y*Rwgradl[1].x + rdf3[1].x*Rwgradl[1].y + 0       *Rwgradl[1].z; 
    Mf3[1].x= Mf3[1].x/2.0;
    Mf3[1].y= Mf3[1].y/2.0;
    Mf3[1].z= Mf3[1].z/2.0;
    
    MfIN[1].x= Mf3[1].x;
    MfIN[1].y= Mf3[1].y;
    MfIN[1].z= Mf3[1].z;
    Mf3[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mf3[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mf3[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mfd3[1].x= 0        *Rwgrad3[1].x - rdf3d[1].z*Rwgrad3[1].y + rdf3d[1].y*Rwgrad3[1].z; 
    Mfd3[1].y= rdf3d[1].z*Rwgrad3[1].x + 0        *Rwgrad3[1].y - rdf3d[1].x*Rwgrad3[1].z; 
    Mfd3[1].z=-rdf3d[1].y*Rwgrad3[1].x + rdf3d[1].x*Rwgrad3[1].y + 0        *Rwgrad3[1].z; 
    MfIN[1].x= Mfd3[1].x;
    MfIN[1].y= Mfd3[1].y;
    MfIN[1].z= Mfd3[1].z;
    Mfd3[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mfd3[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mfd3[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mff3[1].x= 0         *Rwgradlf[1].x - rdf3f[1].z*Rwgradlf[1].y + rdf3f[1].y*Rwgradlf[1].z; 
    Mff3[1].y= rdf3f[1].z*Rwgradlf[1].x + 0        *Rwgradlf[1].y - rdf3f[1].x*Rwgradlf[1].z; 
    Mff3[1].z=-rdf3f[1].y*Rwgradlf[1].x + rdf3f[1].x*Rwgradlf[1].y + 0        *Rwgradlf[1].z; 

    MfIN[1].x= Mff3[1].x;
    MfIN[1].y= Mff3[1].y;
    MfIN[1].z= Mff3[1].z;
    Mff3[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mff3[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mff3[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mf4[1].x= 0        *Rwgradr[1].x - rdf4[1].z*Rwgradr[1].y + rdf4[1].y*Rwgradr[1].z; 
    Mf4[1].y= rdf4[1].z*Rwgradr[1].x + 0        *Rwgradr[1].y - rdf4[1].x*Rwgradr[1].z; 
    Mf4[1].z=-rdf4[1].y*Rwgradr[1].x + rdf4[1].x*Rwgradr[1].y + 0        *Rwgradr[1].z; 
    Mf4[1].x= Mf4[1].x/2.0;
    Mf4[1].y= Mf4[1].y/2.0;
    Mf4[1].z= Mf4[1].z/2.0;

    MfIN[1].x= Mf4[1].x;
    MfIN[1].y= Mf4[1].y;
    MfIN[1].z= Mf4[1].z;
    Mf4[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mf4[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mf4[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mfd4[1].x= 0        *Rwgrad4[1].x - rdf4d[1].z*Rwgrad4[1].y + rdf4d[1].y*Rwgrad4[1].z; 
    Mfd4[1].y= rdf4d[1].z*Rwgrad4[1].x + 0        *Rwgrad4[1].y - rdf4d[1].x*Rwgrad4[1].z; 
    Mfd4[1].z=-rdf4d[1].y*Rwgrad4[1].x + rdf4d[1].x*Rwgrad4[1].y + 0        *Rwgrad4[1].z; 
    MfIN[1].x= Mfd4[1].x;
    MfIN[1].y= Mfd4[1].y;
    MfIN[1].z= Mfd4[1].z;
    Mfd4[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mfd4[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mfd4[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;

    Mff4[1].x= 0         *Rwgradrf[1].x - rdf4f[1].z*Rwgradrf[1].y + rdf4f[1].y*Rwgradrf[1].z; 
    Mff4[1].y= rdf4f[1].z*Rwgradrf[1].x + 0         *Rwgradrf[1].y - rdf4f[1].x*Rwgradrf[1].z; 
    Mff4[1].z=-rdf4f[1].y*Rwgradrf[1].x + rdf4f[1].x*Rwgradrf[1].y + 0         *Rwgradrf[1].z; 

    MfIN[1].x= Mff4[1].x;
    MfIN[1].y= Mff4[1].y;
    MfIN[1].z= Mff4[1].z;
    Mff4[1].x= cos(rotZ)*cos(rotY)                                *MfIN[1].x - sin(rotZ)*cos(rotY)*MfIN[1].y + sin(rotY)*MfIN[1].z ; // % bon
    Mff4[1].y= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].x + (cos(rotZ)*cos(rotX) - sin(rotZ)*sin(rotY)*sin(rotX))*MfIN[1].y - cos(rotY)*sin(rotX)*MfIN[1].z ;
    Mff4[1].z= (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].x + (cos(rotZ)*sin(rotX) + sin(rotZ)*sin(rotY)*cos(rotX))*MfIN[1].y+cos(rotY)*cos(rotX)*MfIN[1].z;
// End "Moment" computation "function" section


    ddx = 1.0/mTot*(            Rwgradn[1].x+Rwgrad[1].x+RwgradLado[1].x+ Rwgradr[1].x + Rwgradl[1].x + Rwgrad1[1].x + Rwgrad2[1].x + Rwgrad3[1].x + Rwgrad4[1].x + 2*Rwgradrf[1].x + 2*Rwgradlf[1].x + th[1].x);  
    ddy = 1.0/mTot*(            Rwgradn[1].y+Rwgrad[1].y+RwgradLado[1].y+ Rwgradr[1].y + Rwgradl[1].y + Rwgrad1[1].y + Rwgrad2[1].y + Rwgrad3[1].y + Rwgrad4[1].y + 2*Rwgradrf[1].y + 2*Rwgradlf[1].y + th[1].y);   
    ddz = 1.0/mTot*(-weight*1.0 + Rwgradn[1].z+Rwgrad[1].z+RwgradLado[1].z+ Rwgradr[1].z + Rwgradl[1].z + Rwgrad1[1].z + Rwgrad2[1].z + Rwgrad3[1].z + Rwgrad4[1].z + 2*Rwgradrf[1].z + 2*Rwgradlf[1].z + th[1].z); // Can Hold it in the air and study it. In a "wind tunnel"


    ddalpha = 0.0;
    ddtheta = cntrlyOld*0.0 + 1.0*( - 1.0/Jy*(Mf1[1].y+Mf2[1].y+Mf3[1].y+Mf4[1].y + Mfd1[1].y+Mfd2[1].y+Mfd3[1].y+Mfd4[1].y+ Mff1[1].y+Mff2[1].y+Mff3[1].y+Mff4[1].y) - 1*dtheta*5.5);  //-5.5 aerodynamic "turning drag"
    ddpsi   = cntrlzOld*0.0 + 1.0*( - 1.0/Jz*(Mf1[1].z+Mf2[1].z+Mf3[1].z+Mf4[1].z + Mfd1[1].z+Mfd2[1].z+Mfd3[1].z+Mfd4[1].z+ Mff1[1].z+Mff2[1].z+Mff3[1].z+Mff4[1].z) - 1*dpsi  *5.5); // */

    ddtheta = cntrlyOld*(1.0-switchLOD) + (switchLOD)*( - 1.0/Jy*(Mf1[1].y+Mf2[1].y+Mf3[1].y+Mf4[1].y + Mfd1[1].y+Mfd2[1].y+Mfd3[1].y+Mfd4[1].y+ Mff1[1].y+Mff2[1].y+Mff3[1].y+Mff4[1].y) - 1*dtheta*5.5);  //-5.5 aerodynamic "turning drag"
    ddpsi   = cntrlzOld*(1.0-switchLOD) + (switchLOD)*( - 1.0/Jz*(Mf1[1].z+Mf2[1].z+Mf3[1].z+Mf4[1].z + Mfd1[1].z+Mfd2[1].z+Mfd3[1].z+Mfd4[1].z+ Mff1[1].z+Mff2[1].z+Mff3[1].z+Mff4[1].z) - 1*dpsi  *5.5); // */
    // Note: cntrl*Old is the pure control signal for the Missile torque. It's what points the missile at the target.
    // The variables "Mf*" are the moments induced by the missile fins which actuates the missile.
    
    // ROTATION MARKOVIAN CALCULATION.
    alphaI = 0*PI/180; 
    dalpha += ddalpha * dt1 ;   
    dalphaI = dalpha + dtheta*sin(alphaI)*tan(thetaI) - dpsi*cos(alphaI)*tan(thetaI) ; 
    thetaI  = thetaI+ dthetaI*dt1;
    // Saturate turn rate here.
    dtheta += ddtheta * dt1;  
    float turnLim = 10*PI/180.0;
     if (abs(dtheta) > turnLim  & switchLOD ==1) { 
      dtheta = dtheta/abs(dtheta)*turnLim;
    } // */
    dthetaI = dtheta*cos(alphaI) + dpsi*sin(alphaI); 
    psiI  = psiI + dpsiI*dt1; 
    dpsi += ddpsi * dt1;     
     if (abs(dpsi) > turnLim  & switchLOD ==1) { 
      dpsi = dpsi/abs(dpsi)*turnLim;
    } // */
    dpsiI = (-dtheta*sin(alphaI)* 1/cos(thetaI) + dpsi*cos(alphaI)*1/cos(thetaI))*1; 

    // Find maximum angular rates through out simulation flight case.
    if (dalphaMAX<abs(dalpha)) {
      dalphaMAX = abs(dalpha);
    }
    if (dthetaMAX<abs(dtheta)) {
      dthetaMAX = abs(dtheta);
    }
    if (dpsiMAX<abs(dpsi)) {
      dpsiMAX = abs(dpsi);
    }
    
        // Find max body frame velocity
    if (ubMAX<abs(ub)) {
      ubMAX = abs(ub);
    }
    if (vbMAX<abs(vb)) {
      vbMAX = abs(vb);
    }
    if (wbMAX<abs(wb)) {
      wbMAX = abs(wb);
    }

    if (alphaI > PI) {  
      alphaI = alphaI - 2 * PI;
    }
    if (alphaI <= -PI) {
      alphaI = alphaI + 2 * PI;
    }  
    if (thetaI > 2*PI) {
      thetaI = thetaI - 2 * PI;
    }
    if (thetaI <= 0) {
      thetaI = thetaI + 2 * PI;
    }
    if (psiI > 2*PI) {
      psiI = psiI - 2 * PI;
    }
    if (psiI <= 0) {
      psiI = psiI + 2 * PI;
    }
    // Extract Error Quaternion
    qx= erqrefqk.x; // Note: This still in quaternion space.
    qy= erqrefqk.y;
    qz= erqrefqk.z;


    stroke(255);  // Paint my letters white. Yes! It worked. 
    // */
    // POSITION MARKOVIAN CALCULATION.
    x =  x + dx*dt1;   
    dx += ddx*dt1;      
    y = y + dy*dt1;   
    dy += ddy*dt1;     
    z = z + dz*dt1;   
    dz += ddz*dt1;   
    
    VM  = abs(Vc1*(1.0-switch3) + Vc3*switch3); //sqrt( pow(dx,2) + pow(dy,2) + pow(dz,2) ); // abs(Vc1);  // Should be VC mmagnitude towards the target.
    VM2 = abs(Vc2);

// Freeze if close.
    if (PosVecNorm<=20) { 
      ddalpha=0;
      ddtheta=0;
      ddpsi=0;
      dalpha=0;
      dtheta=0;
      dpsi=0;
      ddz = 0;
      dz = 0;
      ddx=0;
      ddy=0;
      dx= 0;
      dy=0;
      cntrlx= 0; 
      cntrly= 0;  
      cntrlz= 0;
      cntrlyOld=0;
      weight = 0;
    } // */

  }

  if (guidFlag) {  
    if (abs(b0z)<1.0) { 
      float TEMPzz = (XLAMD1 - XLAMD1m1)/dt1;
      float XNT = Rgo1/TEMPzz;
      YDD= XNT-dwb;
      YD= YD + YDD*dt1;
      Y= Y + YD*dt1;
      float W = PI/2;
      float e = 2.718281828;
      float YTDD = ddtz;
      float YTDDD = dddtz;
       //XNC= (XNP*Vc1*dXLAMD1)/-1.0; // pure nc command
       float VcMagn = sqrt(pow(Vc1,2) + pow(Vc2,2) + pow(Vc3,2));
       tgo = PosVecNorm/(VcMagn+ 0.00001);// No div by zero
       //XNC = - (XNP*1+3)*(Vc1*dXLAMD1);
      //XNC2= (XNP*1000*dXLAMD1r)/-1.0; // pure nc command
      //XNC = XNP*()/pow(tgo,2);
      //XNC2= XNP*()/pow(tgo,2); // pure nc command



       
       cntrly= -Gp2*erqrefqk.y+  -Gr2*dtheta ; 
       cntrlz= -Gp3*erqrefqk.z+  -Gr3*dpsi;  // */
       /*cntrly= -Kp2*erqrefqk.y+  -Kr2*dtheta ; 
       cntrlz= -Kp3*erqrefqk.z+  -Kr3*dpsi; // */
       cntrlyOld = cntrly;
       cntrlzOld = cntrlz;
       

       if (abs(cntrlyOld) > XNCLODMAX){
         XNCLODMAX= abs(cntrlyOld);
       }
       if (abs(cntrlzOld) > XNCLODrMAX){
         XNCLODrMAX= abs(cntrlzOld);
       }

      ZEM[1].x = Rtmx - Vtmx*tgo;
      ZEM[1].y = Rtmy - Vtmy*tgo;
      ZEM[1].z = Rtmz - Vtmz*tgo; // */
      
      float ZEMDotuLOSi = ZEM[1].x* uLOSi[1].x + ZEM[1].y* uLOSi[1].y + ZEM[1].z* uLOSi[1].z;
      ZEMparr[1].x = ZEMDotuLOSi* uLOSi[1].x; // bon.
      ZEMparr[1].y = ZEMDotuLOSi* uLOSi[1].y;
      ZEMparr[1].z = ZEMDotuLOSi* uLOSi[1].z;
      
      ZEMperp[1].x = ZEM[1].x - ZEMparr[1].x;
      ZEMperp[1].y = ZEM[1].y - ZEMparr[1].y;
      ZEMperp[1].z = ZEM[1].z - ZEMparr[1].z;
      
      
      
      
     //#############################################
     // IMPACT ANGLE DESIGNATION STUFF.
          // 1) Calculate the initial "lamb(t=0)"
         // Msl smart bomb guidance logic on.
                              // Offset off of balistTime MUST > dt1 !!!!!!!!!!!!!!!!!!!!!!!
         // Turn impact angle controller on after this time window, para siempre!
     if (tsec<ballistTime+0.5) { // Calculate for  short time then savee and freeze the output.
       XLAMD0 = atan2(-LOSi[1].z, sqrt(pow(LOSi[1].x,2) + pow(LOSi[1].y,2)) );
     } 
     
    textSize(10);
    fill(0, 255, 255);
    text("Impact Angle [deg]", 75, 210, 10); 
     // Print initial angle.
     textSize(10);
     fill(0, 255, 255);
     textSize(20);
     fill(0, 102, 153);
     //text(String.format("%.2f", XLAMD0*180.0/PI), 75, 230, 10); // scd: 4/118/2019, i forgot, what is this?
     
     
     
    // 2) Calculate realtime "lamb(t)"
    // 3) Calculate inertial frame location of command impact angle turn.
      // Below is lamb(t) in [~].   TURN ON IMPACT ANGLE DESIGNATION LOGIC HERE.  
      if (tsec>ballistTime+0.5) { // Calculate for  short time then savee and freeze the output.
         lambImp = atan2(-LOSi[1].z, sqrt(pow(LOSi[1].x,2) + pow(LOSi[1].y,2)) );  // Radians.
         
         if (abs(XLAMDoffset) > 80.0*PI/180.0) {
           XLAMDoffset = 80.0*PI/180.0;
         }
         if (XLAMDoffset < 0.0) {
           XLAMDoffset = 0.0;
         }
         
         XLAMDF = XLAMD0 + XLAMDoffset; // Inertial frame value.  TURN INPACT ANGLE CONTROLL ON!
      } // Default is 0.0  for the above closest variable.
      text(String.format("%.2f", lambImp*180.0/PI), 75, 250, 10); 
    // 4) Calculate the rotated LOS_Imp for Virtual target, driving the impact angle logic.
/*
if tsec > 0.5 + ballistTime { }
//*/
    XLAMDTurn = -(lambImp - XLAMDF) ;    // XLAMDF : "constant", BEWARE THE SIGN.  EMPIRICALLY GUESS FOR RBEST RESULT.
    ver=   1-cos(XLAMDTurn);
    xcomp= LOSi[1].x; // Rotate LOS vector about rw- axis in inertial frame to point at our virtual target.
    ycomp= LOSi[1].y; 
    zcomp= LOSi[1].z;
    kxx=   eurwquat[1].x; // rotate about rw-axis- in inertial frame.
    kyy=   eurwquat[1].y;
    kzz=   eurwquat[1].z;
    LOSiImp[1].x= (kxx*kxx*ver+cos(XLAMDTurn))    *xcomp + (kxx*kyy*ver-kzz*sin(XLAMDTurn))*ycomp + (kxx*kzz*ver+kyy*sin(XLAMDTurn))*zcomp; //Rotate the gradient vector of actuator about vector "k"
    LOSiImp[1].y= (kxx*kyy*ver+kzz*sin(XLAMDTurn))*xcomp + (kyy*kyy*ver+cos(XLAMDTurn))    *ycomp + (kyy*kzz*ver-kxx*sin(XLAMDTurn))*zcomp;
    LOSiImp[1].z= (kxx*kzz*ver-kyy*sin(XLAMDTurn))*xcomp + (kyy*kzz*ver+kxx*sin(XLAMDTurn))*ycomp + (kzz*kzz*ver+cos(XLAMDTurn))    *zcomp; // */
    // 5) Calculate in inertial frame the "virtual" ZEM_Imp.
    // to minus from.   LOSiImp(t) - LOSi(t)
    xcomp= (LOSiImp[1].x - LOSi[1].x); // Rotate LOS vector about rw- axis in inertial frame to point at our virtual target.
    ycomp= (LOSiImp[1].y - LOSi[1].y); 
    zcomp= (LOSiImp[1].z - LOSi[1].z);
    
    // Key for Killing impact angle.
    if ( PosVecNorm < 3000) {
      ckeyImp = 0.0;
    } // */
    
    ZEMperp[1].x += xcomp * ckeyImp;  // Initially 1.0 
    ZEMperp[1].y += ycomp * ckeyImp;
    ZEMperp[1].z += zcomp * ckeyImp;  

    // Project ZEMperp onto body axis of the msl so msl can SEE how much it can affect the decreasing of this vector.
    float tempRW = eurwquat[1].x*ZEMperp[1].x + eurwquat[1].y*ZEMperp[1].y + eurwquat[1].z*ZEMperp[1].z;  // ZEM perp signed magnitude in direction of the msl rw unit vector
    float tempMR = eumrquat[1].x*ZEMperp[1].x + eumrquat[1].y*ZEMperp[1].y + eumrquat[1].z*ZEMperp[1].z; // blah blah mr direction.

/*  float tempImpRW = eurwquat[1].x*xcomp + eurwquat[1].y*ycomp + eurwquat[1].z*zcomp; 
    float tempImpMR = eumrquat[1].x*xcomp + eumrquat[1].y*ycomp + eumrquat[1].z*zcomp; 
    XNC = -XNP*(tempMR)/(pow(tgo,2)+ 0.00001) + (XNP+2)*tempImpRW/(pow(tgo,2)+ 0.00001);
    XNC2= XNP*(tempRW)/(pow(tgo,2) + 0.00001) + (XNP+2)*tempImpMR/(pow(tgo,2)+ 0.00001);

// */

      
      // ZEM perp must be projected on the body frme axis and that used in the fin png.
      XNC = -XNP*(tempMR)/(pow(tgo,2)+ 0.00001);
      XNC2= XNP*(tempRW)/(pow(tgo,2) + 0.00001); // pure nc command


      /*
      float TtoXNCScale =  3000.0/6.0 ;  // *PosVecNorm;
      Autopilot.applyTranslationAndRotation(PVector.mult(new PVector(normVelocity, dwb, dXLAMD1), 1),        
      PVector.mult(new PVector(XNC, XNC2, Vc1*(1.0-switch3) + Vc3*switch3), 1),    // Last value was just Vc1
      PVector.mult(new PVector(Vc2, XNP, dtheta), 1), // To see target acceleration.
      PVector.mult(new PVector(dpsi, cntrlyOld*TtoXNCScale, cntrlzOld*TtoXNCScale), 1),
      PVector.mult(new PVector(switchLOD, 0, 0), 1)); 
      DELq= Autopilot.getDELq();
      DELr= Autopilot.getDELr();  // */
        


      float TtoXNCScale =  3000.0/6.0 ;  // *PosVecNorm;
      Autopilot.applyTranslationAndRotation(PVector.mult(new PVector(normVelocity, dwb, dXLAMD1), 1),        
        PVector.mult(new PVector(dXLAMD1r, rho_sl, Vc1*(1.0-switch3) + Vc3*switch3), 1),    // Last value was just Vc1
        PVector.mult(new PVector(Vc2, XNP, dtheta), 1), // To see target acceleration.
        PVector.mult(new PVector(dpsi, cntrlyOld*TtoXNCScale, cntrlzOld*TtoXNCScale), 1),
        PVector.mult(new PVector(switchLOD, XNC, XNC2), 1)); 
        DELq= Autopilot.getDELq();
        DELr= Autopilot.getDELr();  // */
      
    } else {
    
      DELq=0;
      DELr=0;
      
    }
    
     // Autopilot output.
      finplc1= DELq;
      finplc2= -DELr;
      finplc3= DELq;
      finplc4= -DELr; // */
    
  }
} // End of void draw, the continuous real time simulation loop.

void controlEvent(ControlEvent theEvent) {
  camera.setActive(false);}

void mouseReleased() {
  camera.setActive(true);}

long lastTime = 0; // Time stamp.

void mouseDragged () 
{  
  if (ctlPressed) {    //if (true) then...
    posX = map(mouseX, 0, width, -1, 1);                                       
    posY = map(mouseY, 0, height, -1, 1);
    // Note: (u,v) coordinates of Mouse scaled between -1 and 1.
    // Note: here we set planar position of platform without using the slider controls
  }
}

// Analyse here.
void keyPressed() {    
  if (key == ' ') {
    camera.setRotations(-1.0, 0.0, 0.0); 
    camera.lookAt(8.0, -50.0, 80.0); 
    camera.setDistance(650);      
  } else if (keyCode == CONTROL) {  
    camera.setActive(false);  
    ctlPressed = true;
    llave=  false;  
   
  }

  // BELOW WE SWITCH CERTAIN SIMULATION MODES ON AND OFF.
  else if (key == 'g') {  
    llave=true;
  } else if (key == 'G') {
    llave=false;
  } 
}

void keyReleased() {   
  if (keyCode == CONTROL) {
    camera.setActive(true);
    ctlPressed = false;
  }
}
