/*=============================================================================
  Copyright (C) 2014 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        EventHandling.cpp

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

#include <sstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <EventHandling.h>
#include <EventObserver.h>

#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

// Purpose: Example execution.
//
// Parameter:
// [in ]    std::string cameraID        Use camera with this ID for the example.
void EventHandling::RunExample( std::string cameraID )
{
    VimbaSystem&        sys         = VimbaSystem::GetInstance();                       // Get a reference to the VimbaSystem singleton
    std::cout << "Vimba C++ API Version " << sys << "\n\n";                                    // Print out version of Vimba
    VmbErrorType        err         = sys.Startup();                                    // Initialize the Vimba API
    CameraPtr           pCamera     = CameraPtr();                                      // Our camera

    if( VmbErrorSuccess == err )
    {
        if( cameraID.empty() )                                                          // If no ID was provided use the first camera
        {
            CameraPtrVector cameras;
            err = sys.GetCameras( cameras );
            if( VmbErrorSuccess == err
                    &&  !cameras.empty() )
            {
                err = cameras[0]->Open( VmbAccessModeFull );                            // Open the camera
                if( VmbErrorSuccess == err )
                {
                    pCamera = cameras[0];
                    err = pCamera->GetID( cameraID );
                }
            }
        }
        else
        {
            err = sys.OpenCameraByID( cameraID.c_str(), VmbAccessModeFull, pCamera );   // Open the camera
        }

        if( NULL != pCamera )
        {
            VmbInterfaceType interfaceType;
            pCamera->GetInterfaceType( interfaceType );
            if( VmbErrorSuccess == err )
            {
                switch ( interfaceType )
                {
                case VmbInterfaceEthernet:
                    {
                        if( VmbErrorSuccess == err )
                        {
                            err = DeactivateAllCameraNotifications( pCamera );
                            if( VmbErrorSuccess == err )
                            {
                                err = ActivateNotification( pCamera, "AcquisitionStart" );
                                if( VmbErrorSuccess == err )
                                {
                                    err = RegisterEventObserver( pCamera );
                                    if( VmbErrorSuccess == err )
                                    {
                                        std::cout << "Acquire image to trigger event.\n";
                                        std::cout << "\n----------- Events -----------\n\n";
                                        FramePtr pFrame;
                                        VmbInt32_t exampleTimeoutValue = 2000;
                                        pCamera->AcquireSingleImage( pFrame, exampleTimeoutValue ); // Trigger the event
                                    }
                                }
                            }
                        }
                        break;
                    }
                default:
                    std::cout << "Interface type of camera not supported by this example.: " << cameraID << "\n";    
                }
            }
            else
            {
                std::cout << "Could not get interface type. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
            }
            pCamera->Close();
        }
        else
        {
            std::cout << "Could not open camera or no camera available. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
        }
        sys.Shutdown();
    }
    else
    {
        std::cout << "Could not start system. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
    }
}

// Purpose: Deactivate all camera notification.
//
// Parameter:
// [in ]    CameraPtr   pCamera         Intern camera object.
//
// Returns:
//          VmbErrorSuccess in case of success otherwise an error code
VmbErrorType EventHandling::DeactivateAllCameraNotifications( CameraPtr pCamera )
{
    std::cout << "Deactivate notifications.\n";
    FeaturePtr pFeatureEventSelector;
    VmbErrorType err = pCamera->GetFeatureByName( "EventSelector", pFeatureEventSelector );
    if( VmbErrorSuccess == err )
    {
        EnumEntryVector eventSelectorEntrys;
        err = pFeatureEventSelector->GetEntries( eventSelectorEntrys );
        if( VmbErrorSuccess == err )
        {
            FeaturePtr pFeatureEnumEntry;
            for( size_t i = 0; i < eventSelectorEntrys.size(); i++ )
            {
                std::string entryValue = "";
                err = eventSelectorEntrys[i].GetName( entryValue );
                if( VmbErrorSuccess == err )
                {
                    bool isCurrentEntryAvailable = false;
                    err = pFeatureEventSelector->IsValueAvailable( entryValue.c_str(), isCurrentEntryAvailable );
                    if ( VmbErrorSuccess == err )
                    {
                        if ( isCurrentEntryAvailable )
                        {
                            err = pFeatureEventSelector->SetValue( entryValue.c_str() );
                            if( VmbErrorSuccess == err )
                            {
                                err = pCamera->GetFeatureByName( "EventNotification", pFeatureEnumEntry );
                                if( VmbErrorSuccess == err )
                                {
                                    err = pFeatureEnumEntry->SetValue( "Off" );
                                    if( VmbErrorSuccess != err )
                                    {
                                        std::cout << "Could not set notification 'Off'. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                                    }
                                }
                                else
                                {
                                    std::cout << "Could not get feature 'EventNotification'. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                                }                    
                            }
                            else
                            {
                                std::cout << "Could not set 'EventSelector' value to '" << entryValue << "'. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                            }
                        }
                    }
                    else
                    {
                        std::cout << "Could not check if entry is currently available. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                    }
                }
                else
                {
                    std::cout << "Could not get entry value. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                }
            }
        }
        else
        {
            std::cout << "Could not get 'EventSelector' entry's. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
        }
    }
    else
    {
        std::cout << "Could not get feature 'EventSelector'. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
    }
    return err;
}

// Purpose: Create for each 'Camera->EventData' feature an observer and register it.
//
// Parameter:
// [in ]    CameraPtr   pCamera         Intern camera object.
//
// Returns:
//          VmbErrorSuccess in case of success otherwise an error code
VmbErrorType EventHandling::RegisterEventObserver( CameraPtr pCamera )
{
    FeaturePtrVector features;
    VmbErrorType err = pCamera->GetFeatures( features );
    if( VmbErrorSuccess == err )
    {
        for( size_t i = 0; i < features.size(); i++ )
        {
            std::string category;
            err = features[i]->GetCategory( category );
            if( 0 == category.compare( "/EventControl/EventData" ) && VmbErrorSuccess == err )
            {
                IFeatureObserverPtr pObserver;
                SP_SET( pObserver, new EventObserver( pCamera ) );
                err = features[i]->RegisterObserver( pObserver );
                if( VmbErrorSuccess != err )
                {
                    std::cout << "Could not register observer. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                }
            }
        }
    }
    else
    {
        std::cout << "Could not get features. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
    }
    return err;
}

// Purpose: Activate camera notification.
//
// Parameter:
// [in ]    CameraPtr   pCamera         Intern camera object.
// [in ]    std::string eventName       Name of event to activate.
//
// Returns:
//          VmbErrorSuccess in case of success otherwise an error code
VmbErrorType EventHandling::ActivateNotification( CameraPtr pCamera, std::string eventName  )
{
    std::cout << "Activate notification for '" << eventName << "' events.\n";
    FeaturePtr feature;
    VmbErrorType err = pCamera->GetFeatureByName( "EventSelector", feature );
    if( VmbErrorSuccess == err )
    {
        err = feature->SetValue( eventName.c_str() );
        if( VmbErrorSuccess == err )
        {
            err = pCamera->GetFeatureByName( "EventNotification", feature );
            if( VmbErrorSuccess == err )
            {
                err = feature->SetValue( "On" );
                if( VmbErrorSuccess != err )
                {
                    std::cout << "Could not set notification 'On'. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                }
            }
            else
            {
                std::cout << "Could not get feature 'EventNotification'. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
            }                    
        }
        else
        {
            std::cout << "Could not get selector '" << eventName << "'. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
        }
    }
    else
    {
        std::cout << "Could not get feature 'EventSelector'. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
    }
    return err;
}

}}} // namespace AVT::VmbAPI::Examples