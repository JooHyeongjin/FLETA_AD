/*=============================================================================
  Copyright (C) 2017 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ActionCommands.h

  Description: This example will create an Action Command and send it to any
               camera, given by parameter. The following can be set up with
               parameters as well:
                -send Action Command as broadcast on specific network interface
                -send Action Command as broadcast to all network interfaces
                -send Action Command to specific IP address (unicast)

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

#ifndef AVT_VMBAPI_EXAMPLES_ACTION_COMMANDS
#define AVT_VMBAPI_EXAMPLES_ACTION_COMMANDS

#include "VimbaCPP/Include/VimbaCPP.h"

#include "FrameObserver.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

// struct representing an Action Command
typedef struct tActionCommand
{
    VmbUint32_t     mDeviceKey;
    VmbUint32_t     mGroupKey;
    VmbUint32_t     mGroupMask;

} tActionCommand;

typedef enum tFeatureOwner
{
    eFeatureOwnerUnknown     = 0,
    eFeatureOwnerSystem      = 1,
    eFeatureOwnerInterface   = 2,
    eFeatureOwnerCamera      = 3

} tFeatureOwner;

class cActionCommands
{
    private:
        VimbaSystem&    mSystem;
        InterfacePtr    mInterface;
        CameraPtr       mCamera;

        cFrameObserver* mFrameObserver;

        bool            mSystemFlag;
        bool            mInterfaceFlag;
        bool            mCameraFlag;
        bool            mStreamingFlag;

    public:
        //
        // Constructor
        //
        cActionCommands();

        //
        // Destructor
        //
        ~cActionCommands();

        //
        // Helper method to read feature values of any type
        //
        // Parameters:
        //  [in]        aName   Feature name
        //  [in]        aOwner  Feature owner, like Vimba system, interface or camera
        //  [out]       aType   Feature data type
        //  [in/out]    aValue  Buffer for feature value
        //  [in/out]    aSize   Size of buffer
        //
        VmbErrorType GetFeatureValue( const char* aName, tFeatureOwner aOwner, VmbFeatureDataType* aType, void* aValue, size_t* aSize );

        //
        // Helper method to write feature values of any type
        //
        // Parameters:
        //  [in]    aName   Feature name
        //  [in]    aOwner  Feature owner, like Vimba system, interface or camera
        //  [in]    aType   Feature data type
        //  [in]    aValue  Buffer for feature value
        //  [in]    aSize   Size of buffer
        //
        VmbErrorType SetFeatureValue( const char* aName, tFeatureOwner aOwner, VmbFeatureDataType aType, void* aValue, size_t aSize );

        //
        // Helper method to run a Command Feature
        //
        // Parameters:
        //  [in]    aName   Feature name
        //  [in]    aOwner  Feature owner, like Vimba system, interface or camera
        //
        VmbErrorType RunCommand( const char* aName, tFeatureOwner aOwner );

        //
        // Called when any failure occurs within the example.
        // Ensures to stop streaming, close interface, close camera and shutdown Vimba
        //
        void FailureShutdown();

        //
        // Convert given string to IP address,
        // using respective socket library (Winsock/Arpa)
        //
        // Parameters:
        //  [in]    aString     String to be converted
        //  [out]   aIPAddress  Decimal representation of given IP address string
        //
        VmbErrorType ConvertStringToIPAddress( std::string aString, VmbUint32_t* aIPAddress );

        //
        // Start Vimba and open camera with given string
        //
        // Parameters:
        //  [in]    aCamera     The ID or IP address of the camera to work with
        //
        VmbErrorType PrepareCamera( std::string aCamera );

        //
        // Close camera and shutdown Vimba
        //
        VmbErrorType StopCamera();

        //
        // Prepare trigger settings for given camera
        //
        VmbErrorType PrepareTrigger();

        //
        // Set Action Command information to given feature owner.
        // This could be Vimba system, interface or the camera
        //
        // Parameters:
        //  [in]    aOwner      Feature owner, like System, Interface or Camera
        //  [in]    aCommand    Action Command struct (device key, group key, group mask)
        //
        VmbErrorType PrepareActionCommand( tFeatureOwner aOwner, tActionCommand* aCommand );

        //
        // Prepare streaming settings in Vimba and the camera,
        // like allocating the buffers, start capture engine, etc.
        //
        VmbErrorType PrepareStreaming();

        //
        // End streaming
        //
        VmbErrorType StopStreaming();

        //
        // Send Action Command  on system level.
        // This command will be broadcasted on all network interfaces.
        //
        // Parameters:
        //  [in]    aCamera     The ID or IP address of the camera to work with
        //  [in]    aCommand    Action Command to be used by Vimba and camera
        //
        VmbErrorType SendActionCommandOnAllInterfaces( std::string aCamera, tActionCommand aCommand );

        //
        // Send Action Command on interface level.
        // This command will be broadcasted on given network interface.
        //
        // Parameters:
        //  [in]    aCamera     The ID or IP address of the camera to work with
        //  [in]    aInterface  The network interface on which the Action Command
        //                      will be sent out
        //  [in]    aCommand    Action Command to be used by Vimba and camera
        //
        VmbErrorType SendActionCommandOnInterface( std::string aCamera, std::string aInterface, tActionCommand aCommand );
};

}}} // namespace AVT::VmbAPI::Examples

#endif
