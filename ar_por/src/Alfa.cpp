 /***************************************/
/* Authors: Bas Janssen, Dimitri Waard */
/* Fontys Hogeschool Engineering       */
/* 2017                                */
/*                                     */
/* Modified from Descartes tutorial    */
/***************************************/

#include <math.h>
#include <eigen_conversions/eigen_msg.h>
#include <descartes_planner/dense_planner.h>

//All the letters

// A
EigenSTL::vector_Affine3d createA(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
  //Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }
  //Part 1/3
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step*i , yOffset - 0.5*step*i, 0.0);points.push_back(pose);
  }
  //Part 2/3
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset+ step*10) - step*i , (yOffset-0.5*step*10) - 0.5*step*i, 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset-0.5*step*20, safeOffset);points.push_back(pose);
  }
  //Move the tool to part 3/3
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5),(yOffset-0.5*step*5), safeOffset);points.push_back(pose);
  }
  //Part 3/3
  for (unsigned int i = 0; i<6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5),(yOffset-0.5*step*5)-step*i, 0.0);
    points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5), (yOffset-step*7.5), safeOffset);points.push_back(pose);
  }

  return points;
}
// B
EigenSTL::vector_Affine3d createB(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
  //Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }
  //Part 1/7
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step*i , yOffset, 0.0);points.push_back(pose);
  }
  //Part 2/7
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , yOffset - 0.5 * step*i, 0.0);points.push_back(pose);
  }
  //Part 3/7
  for (unsigned int i = 0; i<60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*7.5) + (cos((M_PI/180.0)*i*3) * (2.5*step)), (yOffset - step * 2.5) - (sin((M_PI/180.0)*i*3) * (2.5*step)), 0.0);points.push_back(pose);
  }
  //Part 4/7
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) , (yOffset - step*2.5)+ 0.5 * step*i, 0.0);points.push_back(pose);
  }  
  //Part 5/7
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) , yOffset - step*i*0.5, 0.0);points.push_back(pose);
  }
  //Part 6/7
  for (unsigned int i = 0; i<60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*2.5) + (cos((M_PI/180.0)*i*3) * (2.5*step)), (yOffset - step * 2.5) - (sin((M_PI/180.0)*i*3) * (2.5*step)), 0.0);points.push_back(pose);
  }
  //Part 7/7
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, (yOffset - step*2.5)+ step*i*0.5, 0.0);points.push_back(pose);
  }  
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }
  return points;
}

//C
EigenSTL::vector_Affine3d createC(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset - step*10, safeOffset);points.push_back(pose);
  }  
  //Part 1/1
  for (unsigned int i = 0; i<60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) - (cos((M_PI/180.0)*i*3) * (5*step)), (yOffset - step*10) + (sin((M_PI/180.0)*i*3) *6*step), 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*10), (yOffset-step*10), safeOffset);points.push_back(pose);
  }

  return points;
}

//D
EigenSTL::vector_Affine3d createD(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }
  //Part 1/2
  for (unsigned int i = 0; i<60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) - (cos((M_PI/180.0)*i*3) * 5 * step), yOffset - (sin((M_PI/180.0)*i*3) * 6 * step), 0.0);points.push_back(pose);
  }
  //Part 2/2
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*10) - step*i , yOffset, 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , yOffset, safeOffset);points.push_back(pose);
  }

  return points;
}

//E
EigenSTL::vector_Affine3d createE(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset - step*5, safeOffset);points.push_back(pose);
  } 
  //Part 1/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, (yOffset- step*5) +step*i, 0.0);points.push_back(pose);
  }
  //Part 2/4
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step*i , yOffset, 0.0);points.push_back(pose);
  }
  //Part 3/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset+ step*10), yOffset- step*i, 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset+ step*10) , (yOffset - step*5), safeOffset);points.push_back(pose);
  }
  //Move the tool to part 4/4
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5),yOffset, safeOffset);points.push_back(pose);
  }
  //Part 4/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step*5, yOffset- 0.5*step*i, 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) , (yOffset - step*3), safeOffset);points.push_back(pose);
  }

  return points;
}

//F
EigenSTL::vector_Affine3d createF(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  } 
//Part 1/3
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step*i , yOffset, 0.0);points.push_back(pose);
  }
  //Part 2/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset+ step*10), yOffset- step*i, 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset+ step*10) , (yOffset - step*5), safeOffset);points.push_back(pose);
  }
  //Move the tool to part 3/3
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5),yOffset, safeOffset);points.push_back(pose);
  }
  //Part 3/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step*5, yOffset- 0.5*step*i, 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) , (yOffset - step*3), safeOffset);points.push_back(pose);
  }

  return points;
}

//G
EigenSTL::vector_Affine3d createG(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*10), yOffset - step*10, safeOffset);points.push_back(pose);
  } 
  //Part 1/3
  for (unsigned int i = 0; i<60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) + (cos((M_PI/180.0)*i*3) * 5 * step), (yOffset - step*10) + (sin((M_PI/180.0)*i*3) * 6*step), 0.0);points.push_back(pose);
  }
  //Part 2/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step*i , (yOffset - step*10), 0.0);points.push_back(pose);
  }
  //Part 3/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) , (yOffset - step*10)+ 0.5*step*i, 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) , (yOffset - step*5), safeOffset);points.push_back(pose);
  }

  return points;
}

//H
EigenSTL::vector_Affine3d createH(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }  
  //Part 1/3
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step * i , yOffset , 0.0);points.push_back(pose);
  } 
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*10) , yOffset , safeOffset);points.push_back(pose);
  } 
  //Move the tool to part 2/3
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5),yOffset, safeOffset);points.push_back(pose);
  }
  //Part 1/3
  for (unsigned int i = 0; i < 7; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5) , yOffset - step * i , 0.0);points.push_back(pose);
  } 
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) , (yOffset - step*6) , safeOffset);points.push_back(pose);
  } 
  //Move the tool to part 3/3
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10),(yOffset - step * 6), safeOffset);points.push_back(pose);
  }
  //Part 3/3
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step * i, (yOffset - step * 6) , 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step*6) , safeOffset);points.push_back(pose);
  }    
  return points;
}

//I
EigenSTL::vector_Affine3d createI(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, (yOffset - step*2.5), safeOffset);points.push_back(pose);
  }  
  //Part 1/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, (yOffset - step * 2.5) - step * i, 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step*7.5), safeOffset);points.push_back(pose);
  }
  //Move the tool to part 2/3
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset,(yOffset - step * 5), safeOffset);points.push_back(pose);
  }
  //Part 2/3
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step * i, (yOffset - step * 5), 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , (yOffset - step*5), safeOffset);points.push_back(pose);
  }
  //Move the tool to part 3/3
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 2.5), safeOffset);points.push_back(pose);
  }
  //Part 3/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 2.5) - step * i, 0.0);points.push_back(pose);
  } 
    //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*10) , (yOffset - step*7.5), safeOffset);points.push_back(pose);
  }   
  return points;
}
//J
EigenSTL::vector_Affine3d createJ(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 2.5), safeOffset);points.push_back(pose);
  }  
  //Part 1/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 2.5) - step * i, 0.0);points.push_back(pose);
  } 
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*10) , (yOffset - step*7.5), safeOffset);points.push_back(pose);
  }   
  //Move the tool to part 2/3
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 5) , safeOffset);points.push_back(pose);
  }  
  //Part 2/3
  for (unsigned int i = 0; i < 16; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - 0.5 * step * i, (yOffset - step * 5), 0.0);points.push_back(pose);
  }  
  //Part 3/3
  for (unsigned int i = 0; i < 60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 2.5) - (sin((M_PI/180.0)*i*3)* 2.5 * step), (yOffset - step * 2.5) - (cos((M_PI/180.0)*i*3) * 2.5 * step), 0.0);points.push_back(pose);
  }  
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 2.5) , yOffset, safeOffset);points.push_back(pose);
  }   
  return points;
}
//K
EigenSTL::vector_Affine3d createK(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }  
  //Part 1/3
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step * i, yOffset, 0.0);points.push_back(pose);
  } 
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , yOffset, safeOffset);points.push_back(pose);
  }  
  //Move the tool to part 2/3
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 5) , safeOffset);points.push_back(pose);
  }  
  //Part 2/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step * i, (yOffset - step * 5) + step * i, 0.0);points.push_back(pose);
  } 
  //Part 3/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5) - step * i, yOffset - step * i, 0.0);points.push_back(pose);
  } 
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step * 5), safeOffset);points.push_back(pose);
  }   
  return points;
}
//L
EigenSTL::vector_Affine3d createL(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), yOffset, safeOffset);points.push_back(pose);
  }  	
  //Part 1/2
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step * i, yOffset, 0.0);points.push_back(pose);
  } 	
  //Part 2/2
  for (unsigned int i = 0; i < 8; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset - step * i, 0.0);points.push_back(pose);
  } 	
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step * 8), safeOffset);points.push_back(pose);
  } 	
  return points;
}
//M
EigenSTL::vector_Affine3d createM(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  } 	
  //Part 1/4
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step * i, yOffset, 0.0);points.push_back(pose);
  } 
  //Part 2/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step * i, yOffset - step * i, 0.0);points.push_back(pose);
  }  
  //Part 3/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5) + step * i, (yOffset - step * 5) - step * i, 0.0);points.push_back(pose);
  }  
  //Part 4/4
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step * i, (yOffset - step * 10), 0.0);points.push_back(pose);
  }     
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step * 10), safeOffset);points.push_back(pose);
  }   
  return points;
}
//N
EigenSTL::vector_Affine3d createN(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }  
  //Part 1/3
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step * i, yOffset, 0.0);points.push_back(pose);
  } 
  //Part 2/3
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step * i, yOffset - 0.7 * step * i, 0.0);points.push_back(pose);
  }   
  //Part 3/3
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step * i, (yOffset - step * 7), 0.0);points.push_back(pose);
  }   
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , (yOffset - step * 7), safeOffset);points.push_back(pose);
  }   
  return points;
}
//O
EigenSTL::vector_Affine3d createO(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
  //Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 5), safeOffset);points.push_back(pose);
  } 
  //Part 1/1
  for (unsigned int i = 0; i<121; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) + (cos((M_PI/180.0)*i*3) * 5*step), (yOffset - step*5) + (sin((M_PI/180.0)*i*3) * 5*step), 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 5), safeOffset);points.push_back(pose);
  }   
  return points;
}
//P
EigenSTL::vector_Affine3d createP(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
  //Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }
  //Part 1/4
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step*i , yOffset, 0.0);points.push_back(pose);
  }
  //Part 2/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , yOffset - 0.5 * step*i, 0.0);points.push_back(pose);
  }
  //Part 3/4
  for (unsigned int i = 0; i<60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*7.5) + (cos((M_PI/180.0)*i*3) * (2.5*step)), (yOffset - step * 2.5) - (sin((M_PI/180.0)*i*3) * (2.5*step)), 0.0);points.push_back(pose);
  }
  //Part 4/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) , (yOffset - step*2.5)+ 0.5 * step*i, 0.0);points.push_back(pose);
  }  
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5), yOffset, safeOffset);points.push_back(pose);
  }	
 return points;
}
//Q
EigenSTL::vector_Affine3d createQ(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
  //Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 5), safeOffset);points.push_back(pose);
  } 
  //Part 1/2
  for (unsigned int i = 0; i<121; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) + (cos((M_PI/180.0)*i*3) * 5*step), (yOffset - step*5) + (sin((M_PI/180.0)*i*3) * 5*step), 0.0);points.push_back(pose);
  }	
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 5), safeOffset);points.push_back(pose);
  }	
  //Move the tool to part 2/2
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5), (yOffset - step * 5) , safeOffset);points.push_back(pose);
  }  
  //Part 2/2
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5) - step * i, (yOffset - step * 5) - step * i, 0.0);points.push_back(pose);
  } 
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step * 10), safeOffset);points.push_back(pose);
  }	  
  return points;
}
//R
EigenSTL::vector_Affine3d createR(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
  //Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }
  //Part 1/5
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step*i , yOffset, 0.0);points.push_back(pose);
  }
  //Part 2/5
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , yOffset - 0.5 * step*i, 0.0);points.push_back(pose);
  }
  //Part 3/5
  for (unsigned int i = 0; i<60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*7.5) + (cos((M_PI/180.0)*i*3) * (2.5*step)), (yOffset - step * 2.5) - (sin((M_PI/180.0)*i*3) * (2.5*step)), 0.0);points.push_back(pose);
  }
  //Part 4/5
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) , (yOffset - step*2.5)+ 0.5 * step*i, 0.0);points.push_back(pose);
  }  
  //Part 5/5
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step*5) - step * i , yOffset - step * i, 0.0);points.push_back(pose);
  }  
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step * 5), safeOffset);points.push_back(pose);
  }	 	
  return points;
}
//S
EigenSTL::vector_Affine3d createS(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }  
  //Part 1/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset - step * i, 0.0);points.push_back(pose);
  }    
  //Part 2/4
  for (unsigned int i = 0; i<60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 2.5)- (cos((M_PI/180.0)*i*3) * 2.5 * step),(yOffset - step * 5) - (sin((M_PI/180.0)*i*3) * 5 * step), 0.0);points.push_back(pose);
  }
  
   //Part 3/4
  for (unsigned int i = 0; i<60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 7.5) - (cos((M_PI/180.0)*i*3) * 2.5 * step),(yOffset - step * 5) + (sin((M_PI/180.0)*i*3) * 5 * step), 0.0);points.push_back(pose);
  }
  //Part 4/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 5) - step * i, 0.0);points.push_back(pose);
  }  
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , (yOffset - step * 10), safeOffset);points.push_back(pose);
  } 
  return points;
}
//T
EigenSTL::vector_Affine3d createT(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step *10), yOffset, safeOffset);points.push_back(pose);
  } 	
  //Part 1/2
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), yOffset - step * i, 0.0);points.push_back(pose);
  }  
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , (yOffset - step * 10), safeOffset);points.push_back(pose);
  } 
  //Move the tool to part 2/2
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 5) , safeOffset);points.push_back(pose);
  } 
  //Part 2/2
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step * i, (yOffset - step * 5), 0.0);points.push_back(pose);
  }   
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step * 5), safeOffset);points.push_back(pose);
  }   
  return points;
}
//U
EigenSTL::vector_Affine3d createU(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), yOffset, safeOffset);points.push_back(pose);
  } 
  //Part 1/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step * i, yOffset, 0.0);points.push_back(pose);
  }    
  //Part 2/3
  for (unsigned int i = 0; i < 60; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5) - (sin((M_PI/180.0)*i*3)* 5 * step), (yOffset - step * 5) + (cos((M_PI/180.0)*i*3) * 5 * step), 0.0);points.push_back(pose);
  }    
  //Part 3/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5) + step * i, (yOffset - step * 10), 0.0);points.push_back(pose);
  }    
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10 ), (yOffset - step * 10), safeOffset);points.push_back(pose);
  }     
  return points;
}
//V
EigenSTL::vector_Affine3d createV(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
  //Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), yOffset, safeOffset);points.push_back(pose);
  }
  //Part 1/3
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step*i , yOffset - 0.5*step*i, 0.0);points.push_back(pose);
  }
  //Part 2/3
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step*i , (yOffset-0.5*step*10) - 0.5*step*i, 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), yOffset-0.5*step*20, safeOffset);points.push_back(pose);
  }
  return points;
}
// W
EigenSTL::vector_Affine3d createW(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), yOffset, safeOffset);points.push_back(pose);
  } 	
  //Part 1/4
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step * i, yOffset - 0.25 * step * i, 0.0);points.push_back(pose);
  }
  //Part 2/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + 0.5 * step * i, (yOffset - 0.25 * step * 10) - 0.5 * step * i, 0.0);points.push_back(pose);
  }  
  //Part 3/4
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + 0.5 * step * 5) - 0.5 * step * i, (yOffset - 0.5 * step * 10) - 0.5 * step * i, 0.0);points.push_back(pose);
  }    
  //Part 4/4
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset+ step * i, (yOffset - 0.75 * step * 10) - 0.25 * step * i, 0.0);points.push_back(pose);
  }  
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), (yOffset - step * 10), safeOffset);points.push_back(pose);
  }  
  return points;
}
//X
EigenSTL::vector_Affine3d createX(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset, yOffset, safeOffset);points.push_back(pose);
  }	
  //Part 1/2
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset + step * i, yOffset - step * i, 0.0);points.push_back(pose);
  }  
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , (yOffset - step * 10), safeOffset);points.push_back(pose);
  }   
  //Move the tool to part 2/2
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), yOffset , safeOffset);points.push_back(pose);
  } 
  //Part 1/2
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) -  step * i , yOffset - step * i, 0.0);points.push_back(pose);
  } 
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step * 10), safeOffset);points.push_back(pose);
  }   
  return points;
}
//Y
EigenSTL::vector_Affine3d createY(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), yOffset, safeOffset);points.push_back(pose);
  }
  //Part 1/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) -  step * i , yOffset - step * i, 0.0);points.push_back(pose);
  }
  //Part 2/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5) +  step * i , (yOffset - step * 5) - step * i, 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , (yOffset - step * 10), safeOffset);points.push_back(pose);
  }   
  //Move the tool to part 2/3
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5), (yOffset - step * 5) , safeOffset);points.push_back(pose);
  }
  //Part 3/3
  for (unsigned int i = 0; i < 6; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 5) -  step * i , (yOffset - step * 5), 0.0);points.push_back(pose);
  }
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step * 5), safeOffset);points.push_back(pose);
  }   
  return points;
}
//Z
EigenSTL::vector_Affine3d createZ(EigenSTL::vector_Affine3d points, float safeOffset, float xOffset, float yOffset, float step)
{
//Offset
  for (unsigned int i = 0; i < 1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10), yOffset, safeOffset);points.push_back(pose);
  }  
  //Part 1/3
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) , yOffset - step * i, 0.0);points.push_back(pose);
  } 
  //Part 2/3
  for (unsigned int i = 0; i < 10; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d((xOffset + step * 10) - step * i , (yOffset - step * 10) + step * i, 0.0);points.push_back(pose);
  } 
  //Part 3/3
  for (unsigned int i = 0; i < 11; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , yOffset - step * i, 0.0);points.push_back(pose);
  }  
  //Move the tool from the worksurface
  for (unsigned int i = 0; i<1; i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(xOffset , (yOffset - step * 10), safeOffset);points.push_back(pose);
  }   
  return points;
}
