/*=============================================================================
  Copyright (C) 2012 - 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ApiController.h

  Description: Implementation file for the ApiController helper class that
               demonstrates how to implement a synchronous single image
               acquisition with VimbaCPP.

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

#ifndef AVT_VMBAPI_EXAMPLES_APICONTROLLER
#define AVT_VMBAPI_EXAMPLES_APICONTROLLER

#include <string>

#include "VimbaCPP/Include/VimbaCPP.h"

#include "CameraObserver.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

class ApiController
{
  public:
    ApiController();
    ~ApiController();

    //
    // Starts the Vimba API and loads all transport layers
    //
    // Returns:
    //  An API status code
    //
    VmbErrorType        StartUp();

    //
    // Shuts down the API
    //
    void                ShutDown();

    //
    // Opens the given camera
    // Sets the maximum possible Ethernet packet size
    // Adjusts the image format
    // Calls the API convenience function to start single image acquisition
    // Closes the camera in case of failure
    //
    // Parameters:
    //  [in]    rStrCameraID        The ID of the camera to work on
    //  [out]   rpFrame             The frame that will be filled. Does not need to be initialized.
    //
    // Returns:
    //  An API status code
    //
    VmbErrorType        AcquireSingleImage( const std::string &rStrCameraID, FramePtr &rpFrame );

    //
    // Gets the width of a frame
    //
    // Returns:
    //  The width as integer
    //
    int                 GetWidth()          const;

    //
    // Gets the height of a frame
    //
    // Returns:
    //  The height as integer
    //
    int                 GetHeight()         const;

    //
    // Gets the pixel format of a frame
    //
    // Returns:
    //  The pixel format as enum
    //
    VmbPixelFormatType  GetPixelFormat()    const;

    //
    // Calls the API convenience function to stop image acquisition
    // Closes the camera
    //
    // Returns:
    //  An API status code
    //
    CameraPtrVector     GetCameraList()     const;
   
    //
    // Returns the frame observer as QObject pointer to connect their signals to the view's slots
    //
    QObject*            GetCameraObserver();

    //
    // Translates Vimba error codes to readable error messages
    //
    // Parameters:
    //  [in]    eErr        The error code to be converted to string
    //
    // Returns:
    //  A descriptive string representation of the error code
    //
    std::string         ErrorCodeToMessage( VmbErrorType eErr ) const;

    //
    // Gets the version of the Vimba API
    //
    // Returns:
    //  The version as string
    //
    std::string         GetVersion() const;

  private:
    // A reference to our Vimba singleton
    VimbaSystem &       m_system;
    // The currently streaming camera
    CameraPtr           m_pCamera;
    // Our camera observer
    CameraObserver*     m_pCameraObserver;
    // The current pixel format
    VmbInt64_t          m_nPixelFormat;
    // The current width
    VmbInt64_t          m_nWidth;
    // The current height
    VmbInt64_t          m_nHeight;
};

}}} // namespace AVT::VmbAPI::Examples

#endif
