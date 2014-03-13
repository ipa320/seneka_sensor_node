#include <Tower.h>

Tower::Tower()
{
	ros::NodeHandle nh_("");
	//ros::ServiceClient client = nh_.serviceClient<...::...>("...");
  //...::... srv;

	bEnable          = false;
 	fVelocity        = 50;
	nTarget_position = 0;
  nStepSize        = 10;
 	eMode            = SINGLE;
 	eDirection       = POSITIVE;
 }

 void setParameters(float fVelocity, int nTarget_position)
 {
 	 	
 }

/*

  srv.request.velocity          = sTrunk.fVelocity;
  srv.request.target_position   = sTrunk.nTarget_position;
  switch (sTrunk.eDirection)
  {
    case positive:
       srv.request.direction    = ...;
       break;
    case negative:
       srv.request.direction    = ...;
       break;
  }
    
  if (sTrunk.bTurnLeft)
  {

  }

  if (sTrunk.bTurnRight)
  {

  }

  if (sTrunk.bEnable)
  {
    
    switch (sTrunk.eMode)
    {
      case single:
        break;

      case custom:
        break;

      case endless:
        break;
    }
  }

  srv.request. ...  = cStarting_time;
  srv.request. ...  = cEnd_time;

  if (client.call(srv))
  {
    return true;
  }
  else
  {
    return false;
  }
}

*/