#include <Geometry.h>

/*
 * This example shows ho you can use the Geometry library to calculate the forward and inverse kinematics of an arbitrary length kinematic chain. The kinematic chain is specified
 * Denavit-Hartenburg parameters, if you're not familiar with them watch this tutorial https://www.youtube.com/watch?v=rA9tm0gTln8 . The forward kinematics calculates the location of the
 * end effector given a set of joint angles defined by D-H parameters stored in the 'chain' member. The inverse kinematics (IK) calculate a set of joint angles given a position for the end effector.
 * In doing so it also sets these angles on the chain member. The D-H parameters in this example are based on my robot arm of which you can find a picture in the sketch folder. The algorithm 
 * can be adapted pretty easily to suit all kinds of different configurations though.
 */

// Link stores the D-H parameters for one link in the chain. It also has a function pointer which defines how the link moves with respect to it's parent, more on that later though
struct Link { float d, theta, r, alpha; void(*move)(Link &, float); };

// KinematicChain manages the links and implements the forward and inverse kinematics
template<int maxLinks> class KinematicChain
{
    // A few variables used in the inverse kinematics defined here to save re-allocating them every time inverse kinematics is called
    Point deltaPose;
    Matrix<maxLinks> deltaAngles;
    Matrix<1,3> inverseJacobian;
    Matrix<3,1> jacobian;
    Transformation currentPose, perturbedPose;

    // The number of links addedto the chain via AddLink
    unsigned int noOfLinks;

  public:
    // An array containing all the D-H parameters for the chain
    Link chain[maxLinks];

    KinematicChain() { noOfLinks = 0; }

    // Add a link - it's D-H parameters as well as a function pointer describing it's joint movement
    void AddLink(float d, float theta, float r, float alpha, void (*move)(Link&, float))
    {
      if(noOfLinks == maxLinks)
        return;
      
      chain[noOfLinks].d = d;
      chain[noOfLinks].theta = theta;
      chain[noOfLinks].r = r;
      chain[noOfLinks].alpha = alpha;
      chain[noOfLinks++].move = move;
    }

    // Transforms pose from the end effector coordinate frame to the base coordinate frame.
    Transformation &ForwardKinematics(Transformation &pose)
    {
      for(int i = noOfLinks - 1; i >= 0; i--)
      {
        // These four operations will convert between two coordinate frames defined by D-H parameters, it's pretty standard stuff
        pose.RotateX(chain[i].alpha);
        pose.Translate(chain[i].r,0,0);
        pose.Translate(0,0,chain[i].d);
        pose.RotateZ(chain[i].theta);
      }

      return pose;
    }

    // Handy overload to save having to feed in a fresh Transformation every time
    Transformation ForwardKinematics()
    {
      currentPose.Reset();
      return ForwardKinematics(currentPose); 
    }

    // Calculates the joints angles Transforms targetPose to the R^chainsize domain of link angles and sets it on the internal chain array
    virtual Transformation &InverseKinematics(Transformation &targetPose, int maxIterations = 1000, float convergenceSpeed = 0.00001, float closeEnough = 0.1, float perturbance = 0.001)
    {
      // Inverse kinematics doesn't have a general solution so we have to solve it iteratively
      for (int it = 0; it < maxIterations; it++)
      {
        // First find the current end effector position
        currentPose.Reset();
        ForwardKinematics(currentPose);

        // And find the error between the target and the current pose
        deltaPose = currentPose.p - targetPose.p;

        // If the search gets close enough then just break
        if (deltaPose.Magnitude() < closeEnough)
          break;

        // Now we move each joint a little bit and see how it affects the position of the end effector.
        for (unsigned int link = 0; link < noOfLinks; link++)
        {
          // Move the link a little bit
          perturbedPose.Reset();
          chain[link].move(chain[link], perturbance);

          // Find the change in end effector position
          ForwardKinematics(perturbedPose);

          // And set the link back to where it was for now
          chain[link].move(chain[link], -perturbance);
          
          // Now divide the change in x/y/z position by the amount by which we moved the joint to find a jacobian in the form of {dx/dtheta, dy/dtheta, dz/dtheta}
          jacobian = (currentPose.p - perturbedPose.p) * (1 / perturbance);

          // Ideally we'd find the pseudoinverse of this jacobian, but that's quite a bit of doing. For this example we'll just use the transpose as an inverse of sorts.
          inverseJacobian = Transpose(jacobian);

          // Now calculate a change in joint angle to bring the chain towards the target position. The joint angle / position relatioship is really non-linear so the
          // jacobian won't be valid very far from the operating point and it's better to move only a little bit at a time; this is handled with the convergeSpeed parameter
          deltaAngles.Set((inverseJacobian * deltaPose) * convergenceSpeed, link);
        }

        // Update the link angles
        for (unsigned int link = 0; link < noOfLinks; link++)
          chain[link].move(chain[link], deltaAngles(link));
      }

      return currentPose;
    }
}; 

// In addition to the D-H parameters, the joint also needs to specify how the D-H parameters change in response to it's movement. This let's the inverse kinematics algorithm know how useful the joints movement is in reaching
// the goal position. It does this through the "move" function pointer which accepts a reference to a link and a float. The function should modify the link's D-H parameters in proportion to the 'amount' parameter. This let's you 
// define any kind of link you like by just writing a function and passing it to AddLink along with the other parameters.

// For example, we can define a revolute joint which changes the theta D-H parameter as it moves
void RevoluteJoint(Link &l, float amount) { l.theta += amount; }

// We could define the same joint but with a little more reluctance to move
void StiffRevoluteJoint(Link &l, float amount) { l.theta += (amount / 2); }

// We can define a prismatic joint which changes the r parameter
void PrismaticJoint(Link &l, float amount) { l.r += amount; }

// Or we could define a joint that doesn't move at all. The IK will effectively ignore this one
void ImmobileJoint(Link&, float) { }

// You get the idea. One thing we can't do is specify joints with more than 1DOF. For ball-and-socket joints and so on, just define multiple 1DOF joints on top of each other (D-H parameters all zero).

void setup()
{
  Serial.begin(115200);

  // We'll start by declaring a chain with up to 5 links
  KinematicChain<5> k;
  Transformation target;

  // Now we'll configure them in accordance with my robot arm. It's joints are all revolute but you can just as easily specify prismatic joint via k.AddLink(d, theta, r, alpha, PrismaticJoint);
  // or whatever else and then let the IK do its thing
  k.AddLink(13, 0, 0, M_PI_2, RevoluteJoint);
  k.AddLink(0, 0, 88, 0, RevoluteJoint);
  k.AddLink(0, 0, 88, 0, RevoluteJoint);
  k.AddLink(18, M_PI_2, 28, M_PI_2, RevoluteJoint);
  k.AddLink(91, 0, 0, 0, RevoluteJoint);

  Serial << "End effector starting position is:\n" << k.ForwardKinematics().p << "\n\n";
  
  target.p.X() = 110;
  target.p.Y() = 80;
  target.p.Z() = 20;

  Serial << "Attempting to set a position of:\n" << target.p << "\n ... this might take a moment\n\n";  
  
  Serial << "Arrived at position: " << k.InverseKinematics(target).p;
}

void loop() { }
