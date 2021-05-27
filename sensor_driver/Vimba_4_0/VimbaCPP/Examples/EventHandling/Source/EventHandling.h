/*=============================================================================
  Copyright (C) 2014 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        EventHandling.h

  Description: The EventHandling example will register observer on all
               'EventData' features and turn on camera notification for
               'AcquisitionStart' events.
  
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

#ifndef AVT_VMBAPI_EXAMPLES_EVENT_HANDLING
#define AVT_VMBAPI_EXAMPLES_EVENT_HANDLING

#include <string>

#include "VimbaCPP/Include/VimbaCPP.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

class EventHandling
{

public:
    // Purpose: Example execution.
    //
    // Parameter:
    // [in ]    std::string cameraID        Use camera with this ID for the example.
    void RunExample( std::string cameraID );

private:
    // Purpose: Deactivate all camera notification.
    //
    // Parameter:
    // [in ]    CameraPtr   pCamera         Intern camera object.
    //
    // Returns:
    //          VmbErrorSuccess in case of success otherwise an error code
    VmbErrorType DeactivateAllCameraNotifications( CameraPtr pCamera );
    
    // Purpose: Create for each 'Camera->EventData' feature an observer and register it.
    //
    // Parameter:
    // [in ]    CameraPtr   pCamera         Intern camera object.
    //
    // Returns:
    //          VmbErrorSuccess in case of success otherwise an error code
    VmbErrorType RegisterEventObserver( CameraPtr pCamera );
    
    // Purpose: Activate camera notification.
    //
    // Parameter:
    // [in ]    CameraPtr   pCamera         Intern camera object.
    // [in ]    std::string eventName       Name of event to activate.
    //
    // Returns:
    //          VmbErrorSuccess in case of success otherwise an error code
    VmbErrorType ActivateNotification( CameraPtr pCamera, std::string eventName );
};

}}} // namespace AVT::VmbAPI::Examples

#endif
