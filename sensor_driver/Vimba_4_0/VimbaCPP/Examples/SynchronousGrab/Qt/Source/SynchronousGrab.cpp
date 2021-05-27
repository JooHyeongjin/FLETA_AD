/*=============================================================================
  Copyright (C) 2012 - 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        SynchronousGrab.cpp

  Description: Qt dialog class for the GUI of the SynchronousGrab example of
               VimbaCPP.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#include <sstream>

#include "SynchronousGrab.h"
using AVT::VmbAPI::FramePtr;
using AVT::VmbAPI::CameraPtrVector;

enum { NUM_COLORS = 3, };
enum { BIT_DEPTH  = 8, };

SynchronousGrab::SynchronousGrab(QWidget *parent, Qt::WindowFlags flags)
    : QMainWindow(parent, flags)
    , m_pApiController( new ApiController() )
{
    ui.setupUi(this);
    ui.m_LabelStream->setAlignment( Qt::AlignCenter );
    // Connect GUI events with event handlers
    QObject::connect( ui.m_ButtonStartStop, SIGNAL( clicked() ), this, SLOT( OnBnClickedButtonStartstop() ));

    // Start Vimba
    VmbErrorType err = m_pApiController->StartUp();
    Log( "Starting Vimba", err );
    if ( VmbErrorSuccess == err )
    {
        std::string WindowText("SynchronousGrab (Qt version) Vimba C++ API Version ");
        WindowText += m_pApiController->GetVersion();
        this->setWindowTitle( QString::fromStdString(WindowText) );
        // Connect new camera found event with event handler
        QObject::connect( m_pApiController->GetCameraObserver(), SIGNAL( CameraListChangedSignal( int )), this, SLOT( OnCameraListChanged( int )));

        // Initially get all connected cameras
        UpdateCameraListBox();
        std::stringstream strMsg;
        strMsg << "Cameras found..." << m_cameras.size();
        Log( strMsg.str() ); 
    }
}

SynchronousGrab::~SynchronousGrab()
{
    // Before we close the application we stop Vimba
    m_pApiController->ShutDown();

    delete m_pApiController;
    m_pApiController = NULL;
}

void SynchronousGrab::OnBnClickedButtonStartstop()
{
    VmbErrorType    err;
    int             nRow = ui.m_ListBoxCameras->currentRow();

    if ( -1 < nRow )
    {
        // Get the frame
        FramePtr pFrame;
        err = m_pApiController->AcquireSingleImage( m_cameras[nRow], pFrame );
        if ( VmbErrorSuccess == err )
        {
            // See if it is not corrupt
            VmbFrameStatusType eReceiveStatus;
            err = pFrame->GetReceiveStatus( eReceiveStatus );
            if (    VmbErrorSuccess == err
                 && VmbFrameStatusComplete == eReceiveStatus )
            {
                // Set up Qt image
                QImage  tmpImage(   m_pApiController->GetWidth(),
                                    m_pApiController->GetHeight(),
                                    VmbPixelFormatRgb8 == m_pApiController->GetPixelFormat() ? QImage::Format_RGB888 : QImage::Format_Indexed8 );

                if(VmbPixelFormatRgb8 != m_pApiController->GetPixelFormat())
                {
                    tmpImage.setNumColors(256);
                    for(int i = 0; i < 256; i++)
                    {
                        tmpImage.setColor(i, qRgb(i, i, i));
                    }
                }
                
                VmbUchar_t *pBuffer;
                err = pFrame->GetImage( pBuffer );
                if ( VmbErrorSuccess == err )
                {
                    VmbUint32_t nSize;
                    err = pFrame->GetImageSize( nSize );
                    if ( VmbErrorSuccess == err )
                    {
                        VmbPixelFormatType ePixelFormat = m_pApiController->GetPixelFormat();
                        if (    VmbPixelFormatMono8 == ePixelFormat
                             || VmbPixelFormatRgb8 == ePixelFormat )
                        {
                            // Copy it
                            // We need that because Qt might repaint the view after we have released the frame already
                            CopyToImage( pBuffer, tmpImage );

                            // Display it
                            QSize s= ui.m_LabelStream->size();
                            ui.m_LabelStream->setPixmap( QPixmap::fromImage( tmpImage ).scaled( s, Qt::KeepAspectRatio) );
                        }
                    }
                }
                
                Log( "Starting Acquisition", err );
                m_bIsStreaming = VmbErrorSuccess == err;
            }
            else
            {
                // If we receive an incomplete image we do nothing
                err = VmbErrorOther;
            }
        }
        Log( "Acquire single image", err );
    }
    else
    {
        Log( "Please select a camera." );
    }

}

//
// This event handler (Qt slot) is triggered through a Qt signal posted by the camera observer
//
// Parameters:
//  [in]    reason          The reason why the callback of the observer was triggered (plug-in, plug-out, ...)
//
void SynchronousGrab::OnCameraListChanged( int reason )
{
    bool bUpdateList = false;

    // We only react on new cameras being found and known cameras being unplugged
    if ( AVT::VmbAPI::UpdateTriggerPluggedIn == reason )
    {
        Log( "Camera list changed. A new camera was discovered by Vimba." );
        bUpdateList = true;
    }
    else if ( AVT::VmbAPI::UpdateTriggerPluggedOut == reason )
    {
        Log( "Camera list changed. A camera was disconnected from Vimba." );
        bUpdateList = true;
    }

    if ( true == bUpdateList )
    {
        UpdateCameraListBox();
    }

    ui.m_ButtonStartStop->setEnabled( 0 < m_cameras.size() || m_bIsStreaming );
}

//
// Copies the content of a byte buffer to a Qt image with respect to the image's alignment
//
// Parameters:
//  [in]    pInbuffer       The byte buffer as received from the cam
//  [out]   OutImage        The filled Qt image
//
void SynchronousGrab::CopyToImage( VmbUchar_t *pInBuffer, QImage &OutImage )
{
    VmbUchar_t *    pCursor = OutImage.bits();
    int             nHeight = m_pApiController->GetHeight();
    int             nWidth  =   QImage::Format_Indexed8 == OutImage.format()
                                ? m_pApiController->GetWidth()
                                : m_pApiController->GetWidth() * NUM_COLORS;

    if ( OutImage.bytesPerLine() != nWidth )
    {
        for ( int y=0; y<nHeight; ++y )
        {
            pCursor = OutImage.scanLine( y );
            for ( int x=0; x<nWidth; ++x )
            {
                *pCursor = *pInBuffer;
                ++pCursor;
                ++pInBuffer;
            }
        }
    }
    else
    {
        memcpy( OutImage.bits(), pInBuffer, nWidth * nHeight );
    }
}

//
// Queries and lists all known camera
//
void SynchronousGrab::UpdateCameraListBox()
{
    // Get all cameras currently connected to Vimba
    CameraPtrVector cameras = m_pApiController->GetCameraList();

    // Simply forget about all cameras known so far
    ui.m_ListBoxCameras->clear();
    m_cameras.clear();

    // And query the camera details again
    for (   CameraPtrVector::const_iterator iter = cameras.begin();
            cameras.end() != iter;
            ++iter )
    {
        std::string         strCameraName;
        std::string         strCameraID;
        if ( VmbErrorSuccess != (*iter)->GetName( strCameraName ))
        {
            strCameraName = "[NoName]";
        }
        // If for any reason we cannot get the ID of a camera we skip it
        if ( VmbErrorSuccess == (*iter)->GetID( strCameraID ))
        {
            ui.m_ListBoxCameras->addItem( QString::fromStdString( strCameraName + " " +strCameraID ) );
            m_cameras.push_back( strCameraID );
        }
    }

    ui.m_ButtonStartStop->setEnabled( 0 < m_cameras.size() || m_bIsStreaming );
}

//
// Prints out a given logging string, error code and the descriptive representation of that error code
//
// Parameters:
//  [in]    strMsg          A given message to be printed out
//  [in]    eErr            The API status code
//
void SynchronousGrab::Log( std::string strMsg, VmbErrorType eErr )
{
    strMsg +=  "..." + m_pApiController->ErrorCodeToMessage( eErr );
    ui.m_ListLog->insertItem( 0, QString::fromStdString( strMsg ) );
}

//
// Prints out a given logging string
//
// Parameters:
//  [in]    strMsg          A given message to be printed out
//
void SynchronousGrab::Log( std::string strMsg )
{
    ui.m_ListLog->insertItem( 0, QString::fromStdString( strMsg ) );
}
