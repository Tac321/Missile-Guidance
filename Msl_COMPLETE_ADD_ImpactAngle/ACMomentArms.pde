// ACMomentArms  Object
// Purpose: This file calculates the moments generated by each fin of the missile.

class ACMomentArms {
  private PVector rotation;  
  private float rotX, rotY, rotZ;
  private PVector rdf1,rdf1d,rdf1f, rdf2,rdf2d,rdf2f,rdf3,rdf3d,rdf3f,rdf4,rdf4d,rdf4f;
  private float inToMeters=  .0254;

  public ACMomentArms(float s) {   
    rotation = new PVector(); 
    rdf1  = new PVector(0, 0, 0);  //bon
    rdf1d = new PVector(0, 0, 0);
    rdf1f = new PVector(0, 0, 0);
    rdf2  = new PVector(0, 0, 0);
    rdf2d = new PVector(0, 0, 0);
    rdf2f = new PVector(0, 0, 0);
    rdf3  = new PVector(0, 0, 0);
    rdf3d = new PVector(0, 0, 0);
    rdf3f = new PVector(0, 0, 0);
    rdf4  = new PVector(0, 0, 0);
    rdf4d = new PVector(0, 0, 0);
    rdf4f = new PVector(0, 0, 0);
    calcQ();  // initially sets Ikine parameters.
  }  // End of the constructor object


  public void applyTranslationAndRotation(PVector rrr) {   
    rotation.set(rrr);
    calcQ(); 
  } // end of apply function


private void calcQ() {     
    quatRotOpsVecProdF(); 
  }

  private void quatRotOpsVecProdF() {  
      // Calculate stuff below
      
    // Fuselage
    rotX= rotation.x;
    rotY= rotation.y;
    rotZ= rotation.z;
   
    
    rdf1.x= cos(rotZ)*cos(rotY) * -23.251*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*5.01*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters ;
    rdf1.y= -sin(rotZ)*cos(rotY)* -23.251*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*5.01*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters ;
    rdf1.z= sin(rotY)*            -23.251*inToMeters+-cos(rotY)*sin(rotX)                             *  5.01*inToMeters+  cos(rotY)*cos(rotX)   *                             0*inToMeters;
    rdf1d.x= cos(rotZ)*cos(rotY) *-30.74*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*5.01*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters; // Dynamic fins.
    rdf1d.y= -sin(rotZ)*cos(rotY)*-30.74*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*5.01*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters;
    rdf1d.z= sin(rotY)*           -30.74*inToMeters+-cos(rotY)*sin(rotX)                             *  5.01*inToMeters+  cos(rotY)*cos(rotX)   *                             0*inToMeters;
    rdf1f.x= cos(rotZ)*cos(rotY) * 18.72*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*5.01*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters;
    rdf1f.y= -sin(rotZ)*cos(rotY)* 18.72*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*5.01*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters;
    rdf1f.z= sin(rotY)*            18.72*inToMeters+-cos(rotY)*sin(rotX)                             *  5.01*inToMeters+  cos(rotY)*cos(rotX)   *                             0*inToMeters;
  
    rdf2.x= cos(rotZ)*cos(rotY) * -23.251*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*5.01*inToMeters ;
    rdf2.y= -sin(rotZ)*cos(rotY)* -23.251*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*5.01*inToMeters ;
    rdf2.z= sin(rotY)*            -23.251*inToMeters+-cos(rotY)*sin(rotX)                             *  0*inToMeters+  cos(rotY)*cos(rotX)   *                             5.01*inToMeters;
    rdf2d.x= cos(rotZ)*cos(rotY) *-30.74*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*5.01*inToMeters;
    rdf2d.y= -sin(rotZ)*cos(rotY)*-30.74*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*5.01*inToMeters;
    rdf2d.z= sin(rotY)*           -30.74*inToMeters+-cos(rotY)*sin(rotX)                             *  0*inToMeters+  cos(rotY)*cos(rotX)   *                             5.01*inToMeters;
    rdf2f.x= cos(rotZ)*cos(rotY) * 18.72*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*5.01*inToMeters;
    rdf2f.y= -sin(rotZ)*cos(rotY)* 18.72*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*5.01*inToMeters;
    rdf2f.z= sin(rotY)*            18.72*inToMeters+-cos(rotY)*sin(rotX)                             *  0*inToMeters+  cos(rotY)*cos(rotX)   *                             5.01*inToMeters;
  
    rdf3.x= cos(rotZ)*cos(rotY) * -23.251*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*-5.01*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters ;
    rdf3.y= -sin(rotZ)*cos(rotY)* -23.251*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*-5.01*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters ;
    rdf3.z= sin(rotY)*            -23.251*inToMeters+-cos(rotY)*sin(rotX)                             *  -5.01*inToMeters+  cos(rotY)*cos(rotX)   *                             0*inToMeters;
    rdf3d.x= cos(rotZ)*cos(rotY) *-30.74*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*-5.01*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters;
    rdf3d.y= -sin(rotZ)*cos(rotY)*-30.74*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*-5.01*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters;
    rdf3d.z= sin(rotY)*           -30.74*inToMeters+-cos(rotY)*sin(rotX)                             *  -5.01*inToMeters+  cos(rotY)*cos(rotX)   *                             0*inToMeters;
    rdf3f.x= cos(rotZ)*cos(rotY) * 18.72*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*-5.01*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters;
    rdf3f.y= -sin(rotZ)*cos(rotY)* 18.72*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*-5.01*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*0*inToMeters;
    rdf3f.z= sin(rotY)*            18.72*inToMeters+-cos(rotY)*sin(rotX)                             *  -5.01*inToMeters+  cos(rotY)*cos(rotX)   *                             0*inToMeters;
  
    rdf4.x= cos(rotZ)*cos(rotY) * -23.251*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*-5.01*inToMeters ;
    rdf4.y= -sin(rotZ)*cos(rotY)* -23.251*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*-5.01*inToMeters ;
    rdf4.z= sin(rotY)*            -23.251*inToMeters+-cos(rotY)*sin(rotX)                             *  0*inToMeters+  cos(rotY)*cos(rotX)   *                             -5.01*inToMeters;
    rdf4d.x= cos(rotZ)*cos(rotY) *-30.74*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*-5.01*inToMeters;
    rdf4d.y= -sin(rotZ)*cos(rotY)*-30.74*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*-5.01*inToMeters;
    rdf4d.z= sin(rotY)*           -30.74*inToMeters+-cos(rotY)*sin(rotX)                             *  0*inToMeters+  cos(rotY)*cos(rotX)   *                             -5.01*inToMeters;
    rdf4f.x= cos(rotZ)*cos(rotY) * 18.72*inToMeters+(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX))*-5.01*inToMeters;
    rdf4f.y= -sin(rotZ)*cos(rotY)* 18.72*inToMeters+(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX))*0*inToMeters + (cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX))*-5.01*inToMeters;
    rdf4f.z= sin(rotY)*            18.72*inToMeters+-cos(rotY)*sin(rotX)                             *  0*inToMeters+  cos(rotY)*cos(rotX)   *                             -5.01*inToMeters;

  }
  


  public PVector getRdf1() {  
    return rdf1; //  
  } // */
  public PVector getRdf1d() { 
    return rdf1d; //  
  } // */
  public PVector getRdf1f() { 
    return rdf1f; //  
  } // */
  public PVector getRdf2() {  
    return rdf2; //  
  } // */
  public PVector getRdf2d() { 
    return rdf2d; //  
  } // */
  public PVector getRdf2f() { 
    return rdf2f; //  
  } // */
  public PVector getRdf3() {  
    return rdf3; //  
  } // */
  public PVector getRdf3d() { 
    return rdf3d; //  
  } // */
  public PVector getRdf3f() { 
    return rdf3f; //  
  } // */
  public PVector getRdf4() {  
    return rdf4; //  
  } // */
  public PVector getRdf4d() { 
    return rdf4d; //  
  } // */
  public PVector getRdf4f() { 
    return rdf4f; //       
  } // */
}
