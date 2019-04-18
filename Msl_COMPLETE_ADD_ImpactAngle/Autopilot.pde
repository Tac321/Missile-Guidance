// Autopilot  Object
// Purpose: This file calculates fin deflections to acheive PNG commanded angular acceleration.


class Autopilot {
// Autopilot Initializations
 private float g = 9.81;
 private float mTot = 45.5;
 private float weight = g*mTot;
 private float LengthFuse = 1.6256; //[m]    64"
 private float DELq=0.0, DELr=0.0 ;
 private float XKDC=1.0;
 private float XK1=1,XK3=1, XKR = 0.6;
 private float rho_sl;
 private float VMap;
 private float CNA, CND, ZA,  ZD,  CMAP,CMA, CMD, XMA, XMD;
 private float  Bal=.1, Q;
 private float SREF=1, SWING, SfTAIL, STAIL, SPLAN;
 private float TMP1, TMP2,TMP3,TMP4;
 private float ALFTR;
 private float WAF, ZAF;
 private float dtheta, dpsi;
 // PNG  
 private float Vc1, Vc2; // {Downrange, Crossrange}, "Vc"
 private float XLAMD1, XLAMD1m1, dXLAMD1,dXLAMD1r,dXLAMD1scd;
 private float XNP=4;
 // More stuff
private PVector LOSi;  // Pursuer-Target LOS.
private PVector uLOSi;
private float Rgo1, Rgo2;
private float RangeLOSNorm1;
private float RangeLOSNorm1m1;
private float dRangeLOSNorm1; // 

private PVector primero, segundo, tercero,cuarto, quinto ;
private float inToMeters=  0.0254;
// AF parameters below.
private float Cd= .47, Cdr=1.15, Cdl=Cdr, Cdyn=Cdl, Cfor=Cdyn,Cdfus=1.00;
private float  radius= 3.5*inToMeters; 
private float  AreaNose= PI*radius*radius ;
private float AreaFuse=radius*2*LengthFuse;
private float Areafwset = 1*(12.5*3*inToMeters*inToMeters)*2; 
private float AreaDyn = 1*2.5*3*(inToMeters*inToMeters);
private float  AreaFor = 1*2.5*(3+1.0/3.0)*(inToMeters*inToMeters); 


private float XNCG=0., XNCGr=0., XNCLOAL=0., XNCLOALr=0.;
private float XNC=0.0, XNC2=0.0;
private float switchLOAL = 0.0;

  public Autopilot(float s) {   
    

  
  primero = new PVector(0, 0, 0); 
    segundo = new PVector(0, 0, 0); 
    tercero = new PVector(0, 0, 0); 
    cuarto = new PVector(0, 0, 0); 
    quinto = new PVector(0, 0, 0); 
    calcQ();  
  } 


  public void applyTranslationAndRotation(PVector aaa, PVector bbb, PVector ccc, PVector ddd, PVector eee) {   
    primero.set(aaa); //Inputs  : normVelocity,dwb,dXLAMD1,dXLAMD1r,rho_sl, Vc1, Vc2, XNP
    segundo.set(bbb);
    tercero.set(ccc);
    cuarto.set(ddd);
    quinto.set(eee);
    calcQ(); 
  } // end of apply function


  private void calcQ() {     
    quatRotOpsVecProdF(); 
  }

  private void quatRotOpsVecProdF() {  
    
    VMap = primero.x;
    dwb = primero.y;
    dXLAMD1= primero.z; // */
    XNC = quinto.y;
    XNC2= quinto.z; // */
    
    dXLAMD1r= segundo.x;
    rho_sl= segundo.y;
    Vc1= segundo.z;
    Vc2= tercero.x;
    XNP= tercero.y;
    dtheta= tercero.z;
    dpsi = cuarto.x;
    XNCLOAL  = cuarto.y;
    XNCLOALr = cuarto.z;
    switchLOAL = quinto.x;
  
    float A= 343;  // [m/s]
    if(VMap<=A){VMap=A+1.0;}  
  
    if(switchLOAL >0.5){
      /*XNCG=  (XNP*Vc1*dXLAMD1)/-1.0; //    [gees]
      XNCGr= (XNP*Vc2*dXLAMD1r)/-1.0; // */
      XNCG= XNC;
      XNCGr= XNC2; // */
    } else{
      XNCG = XNCLOAL ;
      XNCGr= XNCLOALr; // */
    }
    float WGT=weight;
    float DIAM = 2*radius; // [m]
    float XCG=LengthFuse/2; 
    float XHL=(32+30.75)*inToMeters;
    float RHO= rho_sl;   // [kg/m^3]
    SWING  = AreaFor*1/pow(inToMeters,2);
    SfTAIL = Areafwset/2.0 *1/pow(inToMeters,2); 
    STAIL  = AreaDyn*1/pow(inToMeters,2);
    SREF   = PI*DIAM*DIAM/4.*1/pow(inToMeters,2); // [m^2]
    SPLAN=DIAM*LengthFuse*1/pow(inToMeters,2); // [m]
    float XCPB= LengthFuse/2; // XCG;
    float XCPW= XCG-18.73*inToMeters;
    float XMACH=VMap/A;            
    TMP1 =(XCG-XCPW)/DIAM; // [m]
    TMP2 =(XCG-XHL)/DIAM;  // [m]
    TMP3 =(XCG-XCPB)/DIAM;  // 0. [m];
    TMP4 =0;       
    if(XMACH>1){ Bal =sqrt(XMACH*XMACH-1); } 
    else{ Bal = 0.1; }
    Q= .5*RHO*VMap*VMap;    // [kg/m * 1/s^2]
    float P1= WGT*XNCG/(Q*SREF*pow(inToMeters,2) );      
    float P1r= WGT*XNCGr/(Q*SREF*pow(inToMeters,2) );
    float Y1= 8*SWING/(Bal*SREF)+8*STAIL/(Bal*SREF);  
    float Y2= 1.5*SPLAN/SREF;    
    float Y3= 8*STAIL/(Bal*SREF);    
    float Y4= 8*SWING*TMP1/(Bal*SREF)+8*STAIL*TMP2/(Bal*SREF);            
    float Y5= 0.0; 
    float Y6= 8*STAIL*TMP2/(Bal*SREF);
    float P2= Y2-Y3*Y5/Y6;
    float P3= Y1-Y3*Y4/Y6;
    ALFTR= (-P3+sqrt(abs(P3*P3+4.*P2*P1)))/(2.*P2); 
    // float DELqTR= -Y4*ALFTR/Y6-Y5*ALFTR*ALFTR/Y6;
    CNA = 1.5*SPLAN*ALFTR/SREF + 8*SWING/(Bal*SREF) + 8*STAIL/(Bal*SREF);
    CND = 8*STAIL/(Bal*SREF);
    ZA  = -g*Q*SREF*pow(inToMeters,2)*CNA/(WGT*VMap);   
    ZD  = -g*Q*SREF*pow(inToMeters,2)*CND/(WGT*VMap);     
    CMAP= 0 + 8*SWING*TMP1/(Bal*SREF);
    CMA = CMAP+8*STAIL*TMP2/(Bal*SREF);
    CMD = 8*STAIL*TMP2/(Bal*SREF);  
    XMA = Q*SREF*pow(inToMeters,2)*DIAM*CMA/Jy;
    XMD = Q*SREF*pow(inToMeters,2)*DIAM*CMD/Jy; 
    XK1=VMap*(XMA*ZD-XMD*ZA)/(180/PI*g*XMA); 
    XK3=180/PI*g*XK1/VMap; // Used for 3-loop and/or rate gyro loop.
    XKDC= -(1.0-XKR*XK3)/XK1*XKR; 
    DELq=XKR*(XKDC*XNCG+dtheta)*PI/180;  
    DELr=XKR*(XKDC*XNCGr+dpsi)*PI/180; 
    // Saturate fin deflection.
    if (abs(DELq)> maxDeflection) { // Value in degrees for saturation limit.
      DELq=  maxDeflection* DELq/abs(DELq);
    }
    if (abs(DELr)> maxDeflection) {
      DELr=  maxDeflection* DELr/abs(DELr);
    }   
  } 

  public float getDELq() {  
    return DELq; 
  } 
  public float getDELr() { 
    return DELr; 
  } 
  
}