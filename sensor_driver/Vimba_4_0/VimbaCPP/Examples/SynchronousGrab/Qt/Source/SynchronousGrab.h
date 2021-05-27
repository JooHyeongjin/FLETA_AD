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

#ifndef SYNCHRONOUSGRABQT_H
#define SYNCHRONOUSGRABQT_H

#include <QMainWindow>

#include "ui_SynchronousGrab.h"

#include "ApiController.h"

using AVT::VmbAPI::Examples::ApiController;

class SynchronousGrab : public QMainWindow
{
    Q_OBJECT

public:
    SynchronousGrab(QWidget *parent = 0, Qt::WindowFlags flags = 0);
    ~SynchronousGrab();

private:
    Ui::SynchronousGrabClass    ui;
    // Our controller that wraps API access
    ApiController *             m_pApiController;
    // A list of known camera IDs
    std::vector<std::string>    m_cameras;
    // Are we streaming?
    bool                        m_bIsStreaming;
    
    //
    // Queries and lists all known camera
    //
    void UpdateCameraListBox();
    
    //
    // Prints out a given logging string, error code and the descriptive representation of that error code
    //
    // Parameters:
    //  [in]    strMsg          A given message to be printed out
    //  [in]    eErr            The API status code
    //
    void Log( std::string strMsg, VmbErrorType eErr );
    
    //
    // Prints out a given logging string
    //
    // Parameters:
    //  [in]    strMsg          A given message to be printed out
    //
    void Log( std::string strMsg );

    //
    // Copies the content of a byte buffer to a Qt image with respect to the image's alignment
    //
    // Parameters:
    //  [in]    pInbuffer       The byte buffer as received from the cam
    //  [out]   OutImage        The filled Qt image
    //
    void CopyToImage( VmbUchar_t *pInBuffer, QImage &OutImage );

private slots:
    void OnBnClickedButtonStartstop();
    
    //
    // This event handler (Qt slot) is triggered through a Qt signal posted by the camera observer
    //
    // Parameters:
    //  [in]    reason          The reason why the callback of the observer was triggered (plug-in, plug-out, ...)
    //
    void OnCameraListChanged( int reason );    
};

#endif // SYNCHRONOUSGRABQT_H
