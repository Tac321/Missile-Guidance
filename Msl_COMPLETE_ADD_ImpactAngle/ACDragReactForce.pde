// ACDragReactForce  Object

// Purpose: This file calculates the reaction drag forces for all plates of the missile model
// Note: The fuselage of the missile is taken to be a rectangular prism with cubic ends. 




class ACDragReactForce {
  private PVector  unitVel,Stuffs; 
  private PVector Rwgrad,RwgradLado,Rwgradl,Rwgradr,Rwgrad1,Rwgrad2,Rwgrad3,Rwgrad4,Rwgradlf,Rwgradrf,Rwgradn;
  private PVector Grad,GradLado,Gradr,Gradl,Gradd1,Gradd2,Gradd3,Gradd4,Gradn;
  private PVector dragLat, dragLatLado, draglfw,dragrfw,dragnose,dragd1,dragd2,dragd3,dragd4,dragfl,dragfr;
  private float dragLatMagn,dragLatLadoMagn,dragLatMagnr,dragLatMagnl,dragNoseMagn,dragDynMagn,dragForeMagn;
  private float DdotG;
  private float rho_sl=1.225; 
  private float Cd= .07, Cdr=1.15, Cdl=Cdr, Cdyn=Cdl, Cfor=Cdyn,Cdfus=1.00;
  private float  radius= 3.5*.0254; 
  // Plate aras of the missile model are calculated below.
  private float  AreaNose= PI*radius*radius ;
  private float AreaFuse=radius*2*1.6256;
  private float Areafwset = 1*(12.5*3*.0254*.0254)*2; 
  private float AreaDyn = 1*2.5*3*(.0254*.0254);
  private float  AreaFor = 1*2.5*(3+1.0/3.0)*(.0254*.0254); 
  private float xV, yV, zV;
  private float normVelocity;
  
  

  public ACDragReactForce(float s) {   
    
    unitVel = new PVector();
    Stuffs = new PVector();
    
    Grad    = new PVector(0, 0, 0); 
    GradLado= new PVector(0, 0, 0); 
    Gradn   = new PVector(0, 0, 0); 
    Gradr   = new PVector(0, 0, 0); 
    Gradl   = new PVector(0, 0, 0);
    Gradd1  = new PVector(0, 0, 0);
    Gradd2  = new PVector(0, 0, 0);
    Gradd3  = new PVector(0, 0, 0);
    Gradd4  = new PVector(0, 0, 0);
    dragLat= new PVector(0, 0, 0);
    dragLatLado= new PVector(0, 0, 0);
    draglfw= new PVector(0, 0, 0);
    dragrfw= new PVector(0, 0, 0);
    dragnose= new PVector(0, 0, 0);
    dragd1=new PVector(0, 0, 0);
    dragd2=new PVector(0, 0, 0);
    dragd3=new PVector(0, 0, 0);
    dragd4=new PVector(0, 0, 0);
    dragfl=new PVector(0, 0, 0);
    dragfr=new PVector(0, 0, 0);
    Rwgrad = new PVector(1, 1, 1);
    RwgradLado= new PVector(1, 1, 1);
    Rwgradr= new PVector(1, 1, 1);
    Rwgradl= new PVector(1, 1, 1);
    Rwgradn = new PVector(1, 1, 1);
    Rwgrad1= new PVector(1, 1, 1);
    Rwgrad2= new PVector(1, 1, 1);
    Rwgrad3= new PVector(1, 1, 1);
    Rwgrad4= new PVector(1, 1, 1);
    Rwgradlf= new PVector(1, 1, 1);
    Rwgradrf= new PVector(1, 1, 1);
    
    calcQ();  

  } 


 
  public void applyTranslationAndRotation(PVector Stu,PVector unitV,PVector gg,PVector gglad,PVector ggn,PVector ggr,PVector ggl,PVector gdd1,PVector gdd2,PVector gdd3,PVector gdd4) {   
    Stuffs.set(Stu);
    normVelocity= Stuffs.x;
    unitVel.set(unitV);
    xV= unitVel.x;
    yV= unitVel.y;
    zV= unitVel.z;
    Grad.set(gg);
    GradLado.set(gglad);
    Gradn.set(ggn);
    Gradr.set(ggr);
    Gradl.set(ggl);
    Gradd1.set(gdd1);
    Gradd2.set(gdd2);
    Gradd3.set(gdd3);
    Gradd4.set(gdd4);
    
    
    calcQ(); 
  } // end of apply function


private void calcQ() {     
    
    quatRotOpsVecProdF(); 
  }

    
  private void quatRotOpsVecProdF() {  
   
    // Initial drag force magnitude per gradient plate.
    dragLatMagn= abs(.5*rho_sl*normVelocity*normVelocity*Cdfus*AreaFuse) ; 
    dragLatLadoMagn= dragLatMagn*1 ; // fail LOD lol SCD 8/29/2018  ##############################################
    dragLatMagnr= abs(.5*rho_sl*normVelocity*normVelocity*Cdr*Areafwset) ;
    dragLatMagnl= abs(.5*rho_sl*normVelocity*normVelocity*Cdl*Areafwset) ;
    dragNoseMagn= abs(.5*rho_sl*normVelocity*normVelocity*Cd*AreaNose);
    dragDynMagn= abs(.5*rho_sl*normVelocity*normVelocity*Cdyn*AreaDyn);
    dragForeMagn= abs(.5*rho_sl*normVelocity*normVelocity*Cdyn*AreaFor);


  // -- Reaction drag force --
    // fuselage ceil n floor
    dragLat.x = -dragLatMagn*xV ; 
    dragLat.y = -dragLatMagn*yV ; 
    dragLat.z = -dragLatMagn*zV ;
    DdotG = dragLat.x*Grad.x + dragLat.y*Grad.y + dragLat.z*Grad.z; 
    Rwgrad.x= DdotG*Grad.x;
    Rwgrad.y= DdotG*Grad.y;
    Rwgrad.z= DdotG*Grad.z;
     // fuselage wall
    dragLatLado.x = -dragLatLadoMagn*xV ;
    dragLatLado.y = -dragLatLadoMagn*yV ; 
    dragLatLado.z = -dragLatLadoMagn*zV ;
    DdotG = dragLatLado.x*GradLado.x + dragLatLado.y*GradLado.y + dragLatLado.z*GradLado.z;  
    RwgradLado.x= DdotG*GradLado.x;
    RwgradLado.y= DdotG*GradLado.y;
    RwgradLado.z= DdotG*GradLado.z;
    // mige wing static assembly
    dragrfw.x = -dragLatMagnr*xV ; 
    dragrfw.y = -dragLatMagnr*yV ; 
    dragrfw.z = -dragLatMagnr*zV ;
    DdotG = dragrfw.x*Gradr.x + dragrfw.y*Gradr.y + dragrfw.z*Gradr.z; 
    Rwgradr.x= DdotG*Gradr.x;
    Rwgradr.y= DdotG*Gradr.y;
    Rwgradr.z= DdotG*Gradr.z;
    // left wing static assembly
    draglfw.x = -dragLatMagnl*xV ; 
    draglfw.y = -dragLatMagnl*yV ; 
    draglfw.z = -dragLatMagnl*zV ;
    DdotG = draglfw.x*Gradl.x + draglfw.y*Gradl.y + draglfw.z*Gradl.z; 
    Rwgradl.x= DdotG*Gradl.x;
    Rwgradl.y= DdotG*Gradl.y;
    Rwgradl.z= DdotG*Gradl.z;
    //Dynamic fin 1
    dragd1.x = -dragDynMagn*xV;
    dragd1.y = -dragDynMagn*yV;
    dragd1.z = -dragDynMagn*zV;
    DdotG = dragd1.x*Gradd1.x + dragd1.y*Gradd1.y + dragd1.z*Gradd1.z; 
    Rwgrad1.x= DdotG*Gradd1.x;
    Rwgrad1.y= DdotG*Gradd1.y;
    Rwgrad1.z= DdotG*Gradd1.z;

    dragfl.x = -dragForeMagn*xV;
    dragfl.y = -dragForeMagn*yV;
    dragfl.z = -dragForeMagn*zV;
    DdotG = dragfl.x*Gradl.x + dragfl.y*Gradl.y + dragfl.z*Gradl.z;
    Rwgradlf.x= DdotG*Gradl.x;
    Rwgradlf.y= DdotG*Gradl.y;
    Rwgradlf.z= DdotG*Gradl.z; // */

    dragd3.x = -dragDynMagn*xV;
    dragd3.y = -dragDynMagn*yV;
    dragd3.z = -dragDynMagn*zV;
    DdotG = dragd3.x*Gradd3.x + dragd3.y*Gradd3.y + dragd3.z*Gradd3.z; 
    Rwgrad3.x= DdotG*Gradd3.x;
    Rwgrad3.y= DdotG*Gradd3.y;
    Rwgrad3.z= DdotG*Gradd3.z;

    dragd2.x = -dragDynMagn*xV;
    dragd2.y = -dragDynMagn*yV;
    dragd2.z = -dragDynMagn*zV;
    DdotG = dragd2.x*Gradd2.x + dragd2.y*Gradd2.y + dragd2.z*Gradd2.z;
    Rwgrad2.x= DdotG*Gradd2.x;
    Rwgrad2.y= DdotG*Gradd2.y;
    Rwgrad2.z= DdotG*Gradd2.z;
 
    dragfr.x = -dragForeMagn*xV;
    dragfr.y = -dragForeMagn*yV;
    dragfr.z = -dragForeMagn*zV;
    DdotG = dragfr.x*Gradr.x + dragfr.y*Gradr.y + dragfr.z*Gradr.z; //
    Rwgradrf.x= DdotG*Gradr.x;
    Rwgradrf.y= DdotG*Gradr.y;
    Rwgradrf.z= DdotG*Gradr.z;

    dragd4.x = -dragDynMagn*xV;
    dragd4.y = -dragDynMagn*yV;
    dragd4.z = -dragDynMagn*zV;
    DdotG = dragd4.x*Gradd4.x + dragd4.y*Gradd4.y + dragd4.z*Gradd4.z; // 
    Rwgrad4.x= DdotG*Gradd4.x;
    Rwgrad4.y= DdotG*Gradd4.y;
    Rwgrad4.z= DdotG*Gradd4.z;

    dragnose.x = -dragNoseMagn*xV ; 
    dragnose.y = -dragNoseMagn*yV ; 
    dragnose.z = -dragNoseMagn*zV ;
    DdotG = dragnose.x*Gradn.x + dragnose.y*Gradn.y + dragnose.z*Gradn.z;  
    Rwgradn.x= DdotG*Gradn.x;
    Rwgradn.y= DdotG*Gradn.y;
    Rwgradn.z= DdotG*Gradn.z;

  }


  public PVector getRwgrad() { 
    return Rwgrad; 
  } 
  public PVector getRwgradLado() {
    return RwgradLado; 
  } 
  public PVector getRwgradr() {   
    return Rwgradr; //
  } 
  public PVector getRwgradl() { 
    return Rwgradl; //
  } 
  public PVector getRwgradn() { 
    return Rwgradn; // 
  } 
  public PVector getRwgrad1() { 
    return Rwgrad1; // 
  } 
  public PVector getRwgrad2() { 
    return Rwgrad2; // 
  }
  public PVector getRwgrad3() { 
    return Rwgrad3; // 
  } 
  public PVector getRwgrad4() { 
    return Rwgrad4; // 
  }  
  public PVector getRwgradlf() {  
    return Rwgradlf; //  
  } 
  public PVector getRwgradrf() {
    return Rwgradrf; 
  } // */
  
}
