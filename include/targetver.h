// File Location: l:\tests\testgszf\targetver.h
#pragma once

#include <pcl/common/common_headers.h>

#define MIN_INTENSITY 8
// The following macros define the minimum required platform.  The minimum required platform
// is the earliest version of Windows, Internet Explorer etc. that has the necessary features to run 
// your application.  The macros work by enabling all features available on platform versions up to and 
// including the version specified.

// Modify the following defines if you have to target a platform prior to the ones specified below.
// Refer to MSDN for the latest info on corresponding values for different platforms.
#ifndef _WIN32_WINNT            // Specifies that the minimum required platform is Windows Vista.
#define _WIN32_WINNT 0x0600     // Change this to the appropriate value to target other versions of Windows.
#endif

/******************************************************************************
*                              INTERNAL CONSTANTS                             *
******************************************************************************/
#undef M_PI

const double M_PI   = 3.14159265358979323846264338327950288; 
const double K_ToRad  = M_PI / 180.0;

void Log (const char *Xi_pFormat,...);
#include "windows.h"
#include <stdlib.h>
#include <map>
/******************************************************************************
*                              INTERNAL CLASSES                               *
******************************************************************************/
///////////////////////////////////////////////////////////////////////////////
//
//                           SLDGEN_LSRZF_TPoint
//
///////////////////////////////////////////////////////////////////////////////

#pragma once
#pragma pack(1)
struct SLDGEN_LSRZF_TPoint
{
  SLDGEN_LSRZF_TPoint();
  SLDGEN_LSRZF_TPoint(float Xi_X, float Xi_Y, float Xi_Z, BYTE  Xi_Intensity);
  bool GetPolar(float& Xo_Pitch, float& Xo_Yaw, float * Xo_pDist = 0) const;

  float m_X,
        m_Y,
        m_Z;
  BYTE  m_Intensity;
};
#pragma pack()



///////////////////////////////////////////////////////////////////////////////
//
//                           SLDGEN_LSRZF_TLineData
//
///////////////////////////////////////////////////////////////////////////////

struct SLDGEN_LSRZF_TLineData 
{
  SLDGEN_LSRZF_TLineData(int Xi_NumBeams)
    : m_NumBeams(Xi_NumBeams)
    , m_arrPoints(new SLDGEN_LSRZF_TPoint[m_NumBeams])
  {
  }

  ~SLDGEN_LSRZF_TLineData() 
  {
    delete m_arrPoints;
    m_arrPoints = 0;
    m_NumBeams	= 0;
  }

  int m_NumBeams;
  SLDGEN_LSRZF_TPoint * m_arrPoints;
};

///////////////////////////////////////////////////////////////////////////////
//
//                           SLDGEN_LSRZF_CLine
//
///////////////////////////////////////////////////////////////////////////////

struct SLDGEN_LSRZF_CLine
{
SLDGEN_LSRZF_CLine(int Xi_NumBeams)
: m_pImpl(new SLDGEN_LSRZF_TLineData(Xi_NumBeams))
{}

SLDGEN_LSRZF_CLine::~SLDGEN_LSRZF_CLine()
{
  delete m_pImpl;
  m_pImpl = 0;
}

SLDGEN_LSRZF_TPoint& 
SLDGEN_LSRZF_CLine::operator[](int Xi_Idx)
{
  return m_pImpl->m_arrPoints[Xi_Idx];
}

const SLDGEN_LSRZF_TPoint& 
SLDGEN_LSRZF_CLine::operator[](int Xi_Idx) const
{
  return m_pImpl->m_arrPoints[Xi_Idx];
}

int 
SLDGEN_LSRZF_CLine::GetNumBeams() const
{
  return m_pImpl->m_NumBeams;
}

operator const void *() const
{
  return m_pImpl->m_arrPoints;
}

struct SLDGEN_LSRZF_TLineData * m_pImpl;

};

/******************************************************************************
*                           EXPORTED CLASS METHODS                            *
******************************************************************************/
///////////////////////////////////////////////////////////////////////////////
//
//                           SLDGEN_LSRZF_TScan
//
///////////////////////////////////////////////////////////////////////////////
struct SLDGEN_LSRZF_TScan
{
  enum {K_BUFF_SIZE = 4096};
  SLDGEN_LSRZF_TScan() : m_hFile(0) {}
  ~SLDGEN_LSRZF_TScan()
  {
    if(m_hFile)
      ::CloseHandle(m_hFile);
  }
  HANDLE m_hFile;
  char   m_Buff[K_BUFF_SIZE];
#pragma pack(1)
  struct ScanBasicMetadata
  {
    BYTE   m_Version;
    int    m_CountLines;
    int    m_MsrsPerLine;
    float  m_HFOV;
    float  m_YawStartAngle; // degrees
    float  m_HResolution; // degrees
    float  m_VFOV; // degrees
    float  m_PitchStartAngle; // degrees
    float  m_VResolution; // degrees
  } m_MetaData;
#pragma pack()
};

///////////////////////////////////////////////////////////////////////////////
//
//                           SLDGEN_LSRZF_CLaserScan
//
///////////////////////////////////////////////////////////////////////////////
class SLDGEN_LSRZF_CLaserScan
{
public:
/******************************************************************************
*
*: Method name: SLDGEN_LSRZF_CLaserScan
*
******************************************************************************/
SLDGEN_LSRZF_CLaserScan::SLDGEN_LSRZF_CLaserScan ()
: m_pImpl(new SLDGEN_LSRZF_TScan)
{}

/******************************************************************************
*
*: Method name: ~SLDGEN_LSRZF_CLaserScan
*
******************************************************************************/
SLDGEN_LSRZF_CLaserScan::~SLDGEN_LSRZF_CLaserScan ()
{
  delete m_pImpl;
  m_pImpl = 0;
}

/******************************************************************************
*
*: Method name: OpenForRead
*
******************************************************************************/
bool 
SLDGEN_LSRZF_CLaserScan::OpenForRead(const char * Xi_szFileName)
{
  m_pImpl->m_hFile = ::CreateFileA(Xi_szFileName, GENERIC_READ, FILE_SHARE_READ,
                                  NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

  if(m_pImpl->m_hFile == INVALID_HANDLE_VALUE) 
  {
    Log("[SLDGEN_LSRZF_CLaserScan::OpenScan], Failed to open file %s for reading", Xi_szFileName);
    return false;
  }
  DWORD l_ReadBytes;
  if(!ReadFile(m_pImpl->m_hFile, &m_pImpl->m_MetaData, sizeof m_pImpl->m_MetaData, &l_ReadBytes, NULL) || 
     l_ReadBytes != sizeof m_pImpl->m_MetaData)
  {
    Log("[SLDGEN_LSRZF_CLaserScan::OpenScan], Failed to read the version num of the scan");
    return false;
  }
    
  Log("[SLDGEN_LSRZF_CLaserScan::OpenForRead], Metadata of \"%s\": version#: %d, CountLines: %d, MsrsPerLine: %d, HFOV: %lf, YawStartAngle: %lf, HResolution: %lf, VFOV: %lf, PitchStartAngle: %lf, VResolution: %lf", 
    Xi_szFileName, int(m_pImpl->m_MetaData.m_Version), m_pImpl->m_MetaData.m_CountLines, m_pImpl->m_MetaData.m_MsrsPerLine, 
    m_pImpl->m_MetaData.m_HFOV, m_pImpl->m_MetaData.m_YawStartAngle, m_pImpl->m_MetaData.m_HResolution, m_pImpl->m_MetaData.m_VFOV, 
    m_pImpl->m_MetaData.m_PitchStartAngle, m_pImpl->m_MetaData.m_VResolution);

  return true;
}

/******************************************************************************
*
*: Method name: GetVersionID
*
******************************************************************************/
int 
SLDGEN_LSRZF_CLaserScan::GetVersionID() const
{
  return int(m_pImpl->m_MetaData.m_Version);
}

/******************************************************************************
*
*: Method name: GetCountMrmntsPerLine
*
******************************************************************************/
int 
SLDGEN_LSRZF_CLaserScan::GetCountMrmntsPerLine() const
{
  return m_pImpl->m_MetaData.m_MsrsPerLine;
}

/******************************************************************************
*
*: Method name: GetCountLinesPerScan
*
******************************************************************************/
int 
SLDGEN_LSRZF_CLaserScan::GetCountLinesPerScan() const
{
  return m_pImpl->m_MetaData.m_CountLines;
}

/******************************************************************************
*
*: Method name: GetHFOV
*
******************************************************************************/
float 
SLDGEN_LSRZF_CLaserScan::GetHFOV() const
{
  return m_pImpl->m_MetaData.m_HFOV;
}

/******************************************************************************
*
*: Method name: GetInitialYawAngle
*
******************************************************************************/
float 
SLDGEN_LSRZF_CLaserScan::GetInitialYawAngle() const
{
  return m_pImpl->m_MetaData.m_YawStartAngle;
}

/******************************************************************************
*
*: Method name: GetInitialPitchAngle
*
******************************************************************************/
float 
SLDGEN_LSRZF_CLaserScan::GetInitialPitchAngle() const
{
  return m_pImpl->m_MetaData.m_PitchStartAngle;
}

/******************************************************************************
*
*: Method name: GetAngularResolution
*
******************************************************************************/
float 
SLDGEN_LSRZF_CLaserScan::GetAngularResolution() const
{
  return m_pImpl->m_MetaData.m_HResolution;
}

/******************************************************************************
*
*: Method name: GetVFOV
*
******************************************************************************/
float 
SLDGEN_LSRZF_CLaserScan::GetVFOV() const
{
  return m_pImpl->m_MetaData.m_VFOV;
}

/******************************************************************************
*
*: Method name: GetLine
*
******************************************************************************/
bool
SLDGEN_LSRZF_CLaserScan::GetLine(int Xi_LineNum, SLDGEN_LSRZF_CLine * Xo_pLine)
{
  const int l_LineSize(m_pImpl->m_MetaData.m_MsrsPerLine*sizeof SLDGEN_LSRZF_TPoint);
  int l_OffsetToLine(sizeof m_pImpl->m_MetaData + Xi_LineNum*l_LineSize);

  if(Xi_LineNum != -1 && ::SetFilePointer(m_pImpl->m_hFile, l_OffsetToLine, 0, FILE_BEGIN) == INVALID_SET_FILE_POINTER)
  {
    Log("[SLDGEN_LSRZF_CLaserScan::GetLine], SetFilePointer(with %d bytes offset) failed, GetLastError = %d", 
        l_OffsetToLine, GetLastError());
    return false;
  }

  DWORD l_ReadBytes;
  if(!ReadFile(m_pImpl->m_hFile, Xo_pLine->m_pImpl->m_arrPoints, l_LineSize, &l_ReadBytes, NULL) || l_ReadBytes != l_LineSize)
  {
    Log("[SLDGEN_LSRZF_CLaserScan::GetLine], Failed to read the Line data");
    return false;
  }

  return true;
}

/******************************************************************************
*
*: Method name: GetNextLine
*         
******************************************************************************/
bool
SLDGEN_LSRZF_CLaserScan::GetNextLine(SLDGEN_LSRZF_CLine * Xo_pLine)
{
  return GetLine(-1, Xo_pLine);
}

/******************************************************************************
*
*: Method name: ConvertToGSFormat - FYI: done at route recording from raw sensor format
*         
******************************************************************************
bool 
SLDGEN_LSRZF_CLaserScan::ConvertToGSFormat(const char * Xi_ZFSFileName, const char * Xi_GSZFFileName)
{
  int flags = OPEN_DEFAULTFLAGS_LBL;
  if(!ZFSDLL_SetLicense("SDK_GEOSIM", "unlimited", "e194f33e" ) )
  {
    Log("[SLDGEN_LSRZF_CLaserScan::ConvertToGSFormat], SDK license initialization (ZFSDLL_SetLicense) failed");
    return false;
  }

  zfs::zfserr_t err;
  ZFSLinebyline * p = OpenLineByLine(std::string(Xi_ZFSFileName), flags, &err ); // open a zfs scanfile with line by line access
  if( p==NULL ) 
  {
    Log("[SLDC2SNS_ZF_CZFLaser::StoreInGSFormat], OpenLineByLine failed for \"%s\"", Xi_ZFSFileName);
    return false;
  }
  else
  {
    int wm = p->DetectWayMode();
    if( wm>=0 )
      p->InitWay( wm, 0 );
    int px = p->Pixel(); // number of pixel each line
    int ln = p->Lines(); // number of lines (number of lines of all parts)

    // registration
    double matrix[16];
    memset( matrix, 0, sizeof(double)*16 );
    if( p->Matrix().HasTR() ) // Check if available
      p->Matrix().GetTRMatrix4x4( matrix );

    ZFSHeader* firsthead = p->GetFirstZFSHeader(); // access to zfs file header. if scan exist of more scans it is the header of the first scan otherwise it is the same as GetZFSHeader
    ZFSHeader* head = p->GetZFSHeader(); // access to zfs file header. if scan exist of more than one scans it is the head of the last scan
    if( firsthead && head )
    {
      // not relevant
    }
    else
    {
      delete p;
      return false;
    }

    SLDGEN_LSRZF_CLaserScan l_GSScan;
    if(!l_GSScan.OpenForWrite(Xi_GSZFFileName))
    {
      MessageBox(NULL, "Failed to open file for write", "Error", MB_OK|MB_ICONHAND);
      return false;
    }
    double l_YawStart = head->getDouble(67,0.),
      l_YawFinish= head->getDouble(68,0.),
      l_HFOV = l_YawFinish - l_YawStart,
      l_YawAngleResolution = l_HFOV / head->getLong(4,0),
      l_PitchStartAngle = float(head->getLong(61,0))*360.0f / head->getLong(73,0) + head->getDouble(45,0),
      l_PitchEndAngle = float(head->getLong(62,0))*360.0f / head->getLong(73,0) + head->getDouble(45,0);


    int l_Pixels360Width = head->getLong(21,0),
        l_Pixels360Height= l_Pixels360Width/2;

    unsigned char * l_pScanBuff = new unsigned char[l_Pixels360Width * l_Pixels360Height];

    memset(l_pScanBuff, 1, l_Pixels360Width*l_Pixels360Height);

    long l_MaxRef = head->getLong(112,0);

    int minrf = p->MinRf();	// minimum usable intensity
    int maxrf = p->MaxRf(); // maximum intensity of device. You will get higher values if laser is overloaded

    l_GSScan.WriteMetadata(p->Lines(), p->Pixel(), float(l_HFOV), float(l_YawStart), float(l_YawAngleResolution), 
                           float(l_PitchEndAngle - l_PitchStartAngle), float(l_PitchStartAngle), float(360.0/p->Pixel()));

     SLDGEN_LSRZF_CLine l_ScanLine(px);

     p->SkipTo(0); // start with first line
     int maxInt(0);
     int lineheadersize = head->GetLong(7,0) + head->GetLong(8,0);

     bool end = false;
     // all lines
     int akt_n = 0, akt_m = 0;
     for( int nl=0; nl<ln && !end; ++nl) // go trough all lines
     {
       // information about current scanfile if scan consist of more than one part
       int currentpart = p->getCurrentPart();
       int parts = p->getParts();
       int lineInCurrentPart = p->LineInCurrentPart();

       // example get value 44 of lineheader (counter1)
       long pps_line = 0;
       if( lineheadersize>=48 )
         pps_line = *(long*)((char*)(p->L()->LPtr())+44); // Pointer to lineheader

       // go trough all pixel in current line
       for(int i(0); i<px; ++i)
       {
         p->Set(i);				// set pointer to pixel
         if( p->IsSet(i) )   // Check if pixel is enabled. Filterfunction will disable bad points
           // If you want check masks stored in scan as well, you have to open scan with flag OPEN_READ_MASKS
         {
           int rf = p->Rf();		// raw intensity depends on imager typ
            if(rf > maxInt)
              maxInt = rf;
           
            double l_refNorm = p->RfNorm1();
  
           double x,y,z;
           p->XYZ( x,y,z ); // unit=m

           l_ScanLine[i] = SLDGEN_LSRZF_TPoint(float(x),float(y),float(z), BYTE(l_refNorm*255)); // BYTE(double(rf*255)/l_MaxRef)

           // If you need Polarcoordinate instead of cartesian, use following after XYZ
           double elev,hor,range;// [rad,rad,meter]
           ZFModel::XYZ2polar( x,y,z, elev,hor,&range );

           if(hor > 2*M_PI) 
             hor -= 2*M_PI;
           int l_LineIdx = int(hor * (l_Pixels360Width - 1)/2/M_PI +0.5 - 1);
           int l_BeamIdx = l_Pixels360Height - 1 - int(elev * (l_Pixels360Height - 1)/M_PI + 0.5 - 1);
           l_pScanBuff[l_BeamIdx*l_Pixels360Width + l_LineIdx] = BYTE(l_refNorm*255);//BYTE(double(rf*255)/l_MaxRef);
         }
         else
         {
           l_ScanLine[i] = SLDGEN_LSRZF_TPoint(0.0f,0.0f,0.0f, 0);
         }
       }

       l_GSScan.WriteScanLine(&l_ScanLine);
       p->LoadLines(1); // load next line. Use 1 for loading every line
     }

      delete p; // close scanfile
    }

  return true;
}*/

/******************************************************************************
*                             Protected methods                               *
******************************************************************************/

/******************************************************************************
*
*: Method name: OpenForWrite - FYI: done at route recording
*         
******************************************************************************/
bool 
OpenForWrite(const char * Xi_szFileName)
{
  m_pImpl->m_hFile = ::CreateFileA(Xi_szFileName, GENERIC_WRITE, FILE_SHARE_READ,
                                  NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

  if(m_pImpl->m_hFile == INVALID_HANDLE_VALUE) 
  {
    Log("[SLDGEN_LSRZF_CLaserScan::OpenForWrite], Failed to open file %s for writing", Xi_szFileName);
    return false;
  }

  return true;
}


/******************************************************************************
*
*: Method name: WriteMetadata - FYI: done at route recording
*         
******************************************************************************/
bool 
WriteMetadata(int Xi_CountLines, int Xi_MsrsPerLine, float Xi_HFOV,
                                       float Xi_YawStartAngle, float Xi_HResolution, float Xi_VFOV, 
                                       float Xi_PitchStartAngle, float Xi_VResolution)
{
  m_pImpl->m_MetaData.m_Version = 1;
  m_pImpl->m_MetaData.m_CountLines = Xi_CountLines;
  m_pImpl->m_MetaData.m_MsrsPerLine = Xi_MsrsPerLine;
  m_pImpl->m_MetaData.m_HFOV = Xi_HFOV;
  m_pImpl->m_MetaData.m_YawStartAngle = Xi_YawStartAngle;
  m_pImpl->m_MetaData.m_HResolution = Xi_HResolution;
  m_pImpl->m_MetaData.m_VFOV = Xi_VFOV;
  m_pImpl->m_MetaData.m_PitchStartAngle = Xi_PitchStartAngle;
  m_pImpl->m_MetaData.m_VResolution = Xi_VResolution;

  ::SetFilePointer(m_pImpl->m_hFile, 0, 0, FILE_BEGIN);
  DWORD l_Written;
  if(!::WriteFile(m_pImpl->m_hFile, &m_pImpl->m_MetaData, sizeof m_pImpl->m_MetaData, &l_Written, NULL) ||
     l_Written != sizeof m_pImpl->m_MetaData)
  {		
    Log("[SLDGEN_LSRZF_CLaserScan::FlushMetadata], Failed to write to file");
    return false;
	}
  return true;
}

/******************************************************************************
*
*: Method name: WriteScanLine - FYI: done at route recording
*         
******************************************************************************/
bool 
WriteScanLine(const SLDGEN_LSRZF_CLine * Xi_pLine)
{
  DWORD l_Written;
  const DWORD l_RequiredBytes(Xi_pLine->GetNumBeams()*sizeof(SLDGEN_LSRZF_TPoint));
  if(!::WriteFile(m_pImpl->m_hFile, Xi_pLine->operator const void *(), l_RequiredBytes, &l_Written, NULL) ||
     l_Written != l_RequiredBytes)
  {		
    Log("[SLDGEN_LSRZF_CLaserScan::FlushMWriteScanLineetadata], Failed to write to file");
    return false;
	}  
  return true;
}
protected:
/******************************************************************************
*                             Protected members                               *
******************************************************************************/
struct SLDGEN_LSRZF_TScan * m_pImpl;
};



pcl::PointCloud<pcl::PointXYZ>::Ptr  initTargetVer(int argc, char* argv[]);
