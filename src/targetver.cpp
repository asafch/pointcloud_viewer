#include "../include/targetver.h"
#include "../include/ofApp.h"

//function define
void testgszf( const char* filename, SLDGEN_LSRZF_CLaserScan& l_ZFScan, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int Round(double Xi_a);
void Log (const char *Xi_pFormat,...);

#define maxDistance 5000


ofFloatColor getColor(ofPoint point)
{
	float max = 	point.lengthSquared();

		double r,g,b;
		double color =   (max / (maxDistance/13));
		if (color < 1)
		{
			r = (1 - color) ;
			g = color;
			b = 0;
		}
		else if (color < 2)
		{
			color--;
			r = 0;
			g = 1 - color;
			b = color;
		}
		else
		{
			r = 0;
			g = 0;
			b = 1;
		}

		return ofFloatColor(r,g,b);
}

ofMesh* pclNodesToPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr vec)
{
	Points info;
	info.numberOfPoints = vec->points.size();
	//info.points = (float*)(malloc(3*sizeof(float) *  vec->points.size()));
	//info.colors = (float*)(malloc(3*sizeof(float) *  vec->points.size()));

	ofMesh* mesh = new ofMesh();
		  mesh->setMode(OF_PRIMITIVE_POINTS);

	unsigned int index = 0;
	unsigned int indexColor = 0;
	for (int i = 0; i < vec->points.size(); i++)
	{
		/*info.points[index++] = vec->points[i].x;
		info.points[index++] = vec->points[i].y;
		info.points[index++] = vec->points[i].z;
				
		info.colors[indexColor++] = 1;
		info.colors[indexColor++] = 1;
		info.colors[indexColor++] = 1;*/
		ofPoint point( vec->points[i].x,  vec->points[i].y,  vec->points[i].z);
		mesh->addVertex(point);
		mesh->addColor(getColor(point));

	}
	return mesh;
}

void  initTargetVer(const char* filename,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	SLDGEN_LSRZF_CLaserScan l_ZFScan;

	bool l_Ok = l_ZFScan.OpenForRead(filename);

	testgszf(filename, l_ZFScan, cloud);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr  initTargetVer(int argc, char* argv[])
{

	SLDGEN_LSRZF_CLaserScan l_ZFScan;

	bool l_Ok = argc==3 && l_ZFScan.OpenForRead(argv[1]);
	if(!l_Ok)
	{
	 printf("Could not read scan %s", argc==2? argv[1] : "???");
	 return NULL;
	}
//	return testgszf(argv[1], l_ZFScan);
}






void testgszf(const char* filename, SLDGEN_LSRZF_CLaserScan& l_ZFScan, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{


  int l_nRawBeams		= l_ZFScan.GetCountMrmntsPerLine();
  int l_nBeams			= l_nRawBeams / 2;
  
  double l_AngRes		= l_ZFScan.GetAngularResolution()	*	K_ToRad; 
  double l_LineAng0		= l_ZFScan.GetInitialYawAngle()		*	K_ToRad;
  double l_Pitch0		= l_ZFScan.GetInitialPitchAngle()	*	K_ToRad;
  
  int l_LineHalfScan	= Round(M_PI/l_AngRes); // first line in second half
  int l_nLines			= l_LineHalfScan * 2;

  // TODO: Store the scan parameters
  // FYI: this is the relevant Geosim code:
  //m_Params.SetAngleRes(l_AngRes);
  //m_Params.SetLineAngOffset(0);  // master scan is from Left. Left base angle is 0
  //m_Params.SetBeamsAndLines(l_nBeams, l_nLines);
  //m_Params.SetUnits(-1, (float) l_AngRes, 1); // AngleCycle, Xi_AngleLSB, Xi_RangeLSB

  int l_LineNo	=	0;
  int l_Legal	=	0;


  	int indexPoint = 0;
	int indexColor = 0;

	size_t numberOfDots = (l_nBeams * l_nLines);
	size_t divTen = numberOfDots / 10;
	size_t counter = 0;
	int		precent = 0;
	printf("Num of dots: %d\n", (l_nBeams * l_nLines));
  for (int l_iL = 0;	l_iL < l_nLines;	l_iL++)
  {
    int l_iRL		= l_nLines - l_iL - 1; // it turned out the ZF rotates opossite to Riegle
    int l_iRawLine	= l_iRL % l_LineHalfScan;
    
	SLDGEN_LSRZF_CLine l_ZFLine(l_nRawBeams);
    bool l_Ok = l_ZFScan.GetLine(l_iRawLine, &l_ZFLine);
    if(l_Ok)
    {
      double l_LineAng = l_LineAng0 + l_iL * l_AngRes; 
      for (int l_iB = 0; l_iB < l_nBeams; l_iB ++)
      {
        int l_iRawBeam;
        if(l_iRL<l_LineHalfScan)
        {
          l_iRawBeam=l_nBeams-l_iB-1;
        } 
        else
        {
          l_iRawBeam=l_iB + l_nBeams;
        }
    
		// TODO: Store a "point"
        // FYI: this is the relevant Geosim code:
    
		if (counter++ %divTen == 0)
		{
					printf("\r%d%%", (precent));
					precent+=10;
		}
		
		double max =  l_ZFLine[l_iRawBeam].m_X* l_ZFLine[l_iRawBeam].m_X +  l_ZFLine[l_iRawBeam].m_Y *  l_ZFLine[l_iRawBeam].m_Y +  l_ZFLine[l_iRawBeam].m_Z *  l_ZFLine[l_iRawBeam].m_Z;
		
		if (max > maxDistance ||  l_ZFLine[l_iRawBeam].m_Intensity < MIN_INTENSITY)
			continue;
		
		
		   pcl::PointXYZ point;

		
		

		//double r,g,b;
		//double color =   (max / (maxDistance/20));
		//if (color < 1)
		//{
		//	r = (1 - color) ;
		//	g = color;
		//	b = 0;
		//}
		//else if (color < 2)
		//{
		//	color--;
		//	r = 0;
		//	g = 1 - color;
		//	b = color;
		//}
		//else
		//{
		//	r = 0;
		//	g = 0;
		//	b = 1;
		//}

	point.x =  l_ZFLine[l_iRawBeam].m_X;
      point.y =  l_ZFLine[l_iRawBeam].m_Y;
      point.z = l_ZFLine[l_iRawBeam].m_Z;

	 //  uint32_t rgb = (static_cast<uint32_t>(r*255) << 16 |
  //          static_cast<uint32_t>(g * 255) << 8 | static_cast<uint32_t>(b * 255));
  //        point.rgb = *reinterpret_cast<float*>(&rgb);

	   cloud->points.push_back (point);

		//l_pPoint->m_BeamAng = (float)(l_Pitch0 + (l_nBeams-l_iB-1)*l_AngRes - M_PI/2);
        //l_pPoint->m_LineAng = (float)(l_LineAng);
        //l_pPoint->m_Radius = -100.f;
        //l_pPoint->m_X = l_ZFLine[l_iRawBeam].m_X;
        //l_pPoint->m_Y = l_ZFLine[l_iRawBeam].m_Y;
        //l_pPoint->m_Z = l_ZFLine[l_iRawBeam].m_Z;
        //int l_v = l_pPoint->m_Intensity = l_ZFLine[l_iRawBeam].m_Intensity;

      }
   }
    else
    {
      printf("Could not read line %d from %s\n", l_iRawLine, filename);
    }
  }

  printf("\n");
}



/******************************************************************************
*                            INTERNAL FUNCTIONS                               *
******************************************************************************/
int Round(double Xi_a)
{  
  int l_Ret = (Xi_a < 0) ? (int)(Xi_a - 0.5) : (int)(Xi_a + 0.5);
  return l_Ret;
};

void Log (const char *Xi_pFormat,...)
{
}


 SLDGEN_LSRZF_TPoint::SLDGEN_LSRZF_TPoint()
{}

SLDGEN_LSRZF_TPoint::SLDGEN_LSRZF_TPoint(float Xi_X, float Xi_Y, float Xi_Z, BYTE  Xi_Intensity)
: m_X(Xi_X), m_Y(Xi_Y), m_Z(Xi_Z), m_Intensity(Xi_Intensity)
{
}

bool SLDGEN_LSRZF_TPoint::GetPolar(float& Xo_Pitch, float& Xo_Yaw, float * Xo_pDist) const
{
	Xo_Yaw = Xo_Pitch = 0.0;

	float d = sqrt(m_X*m_X + m_Y*m_Y);
	if (d>0.)
	{
		Xo_Pitch = atan( m_Z / d );
		Xo_Pitch += float(0.5 * M_PI);

		Xo_Yaw = float(0.5 * M_PI) - atan2(m_Y, m_X );

		if (Xo_Yaw<0.)	
			Xo_Yaw = float(2 * M_PI) + Xo_Yaw;
	
		if (Xo_pDist)
			*Xo_pDist = sqrt(m_X * m_X	+	m_Y * m_Y	+  m_Z * m_Z);

		return true;
	}
	return false;
}
