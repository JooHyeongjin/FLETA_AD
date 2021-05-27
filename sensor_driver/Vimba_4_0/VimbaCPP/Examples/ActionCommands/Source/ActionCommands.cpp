/*=============================================================================
  Copyright (C) 2017 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ActionCommands.cpp

  Description: see header file for description

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

#include <iostream>
#include <string.h>

#include <ActionCommands.h>

// socket library to check IP addresses
#ifdef _WIN32
    #include <conio.h>
    #include <WinSock.h>
#else
    #include <arpa/inet.h>
#endif

// number of frame buffers to be used by example
#define NUM_FRAMES 3

namespace AVT {
namespace VmbAPI {
namespace Examples {

//
// Constructor
//
cActionCommands::cActionCommands()
    : mSystem( VimbaSystem::GetInstance() )
    , mFrameObserver( NULL )
    , mSystemFlag( false )
    , mInterfaceFlag( false )
    , mCameraFlag( false )
    , mStreamingFlag( false )
{
}

//
// Destructor
//
cActionCommands::~cActionCommands()
{
}

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
VmbErrorType cActionCommands::GetFeatureValue( const char* aName, tFeatureOwner aOwner, VmbFeatureDataType* aType, void* aValue, size_t* aSize )
{
    VmbErrorType lError = VmbErrorSuccess;

    // check parameter
    if( (NULL == aName) || (NULL == aValue) )
    {
        return VmbErrorBadParameter;
    }

    // get feature pointer
    FeaturePtr lFeature;
    switch( aOwner )
    {
        case eFeatureOwnerSystem:
        {
            lError = this->mSystem.GetFeatureByName( aName, lFeature );
            break;
        }
        case eFeatureOwnerInterface:
        {
            lError = this->mInterface->GetFeatureByName( aName, lFeature );
            break;
        }
        case eFeatureOwnerCamera:
        {
            lError = this->mCamera->GetFeatureByName( aName, lFeature );
            break;
        }
        default:
        {
            lError = VmbErrorBadParameter;
            break;
        }
    }

    // get feature data type
    VmbFeatureDataType lType = VmbFeatureDataUnknown;
    lError = lFeature->GetDataType( lType );
    if( (VmbErrorSuccess == lError) && (NULL != aType) )
    {
        *aType = lType;
    }

    // proceed in case of no error
    if( VmbErrorSuccess == lError )
    {
        // set feature value
        switch( lType )
        {
            case VmbFeatureDataInt:
            {
                // get value
                VmbInt64_t lValue = 0;
                lError = lFeature->GetValue( lValue );
                if( VmbErrorSuccess == lError )
                {
                    memcpy( aValue, &lValue, sizeof(VmbInt64_t) );
                }

                // set buffer size
                if( NULL != aSize )
                {
                    *aSize = sizeof(VmbInt64_t);
                }

                break;
            }
            default:
            {
                lError = VmbErrorBadParameter;
                break;
            }
        }
    }

    return lError;
}

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
VmbErrorType cActionCommands::SetFeatureValue( const char* aName, tFeatureOwner aOwner, VmbFeatureDataType aType, void* aValue, size_t aSize )
{
    VmbErrorType lError = VmbErrorSuccess;

    // check parameter
    if( (NULL == aName) || (NULL == aValue) )
    {
        return VmbErrorBadParameter;
    }

    // get feature pointer
    FeaturePtr lFeature;
    switch( aOwner )
    {
        case eFeatureOwnerSystem:
        {
            lError = this->mSystem.GetFeatureByName( aName, lFeature );
            break;
        }
        case eFeatureOwnerInterface:
        {
            lError = this->mInterface->GetFeatureByName( aName, lFeature );
            break;
        }
        case eFeatureOwnerCamera:
        {
            lError = this->mCamera->GetFeatureByName( aName, lFeature );
            break;
        }
        default:
        {
            lError = VmbErrorBadParameter;
            break;
        }
    }

    // check feature data type
    VmbFeatureDataType lType = VmbFeatureDataUnknown;
    lFeature->GetDataType( lType );
    if( lType != aType )
    {
        return VmbErrorBadParameter;
    }

    // proceed in case of no error
    if( VmbErrorSuccess == lError )
    {
        // set feature value
        switch( aType )
        {
            case VmbFeatureDataInt:
            {
                // check size
                if( sizeof(VmbInt64_t) >= aSize )
                {
                    // set value
                    VmbInt64_t lValue = (VmbInt64_t)aValue;
                    lError = lFeature->SetValue( lValue );
                }

                break;
            }
            case VmbFeatureDataEnum:
            {
                // set value
                const char* lValue = (char*)aValue;
                lError = lFeature->SetValue( lValue );

                break;
            }
            default:
            {
                lError = VmbErrorBadParameter;
                break;
            }
        }
    }

    return lError;
}

//
// Helper method to run a Command Feature
//
// Parameters:
//  [in]    aName   Feature name
//  [in]    aOwner  Feature owner, like Vimba system, interface or camera
//
VmbErrorType cActionCommands::RunCommand( const char* aName, tFeatureOwner aOwner )
{
    VmbErrorType lError = VmbErrorSuccess;

    // get feature pointer
    FeaturePtr lFeature;
    switch( aOwner )
    {
        case eFeatureOwnerSystem:
        {
            lError = this->mSystem.GetFeatureByName( aName, lFeature );
            break;
        }
        case eFeatureOwnerInterface:
        {
            lError = this->mInterface->GetFeatureByName( aName, lFeature );
            break;
        }
        case eFeatureOwnerCamera:
        {
            lError = this->mCamera->GetFeatureByName( aName, lFeature );
            break;
        }
        default:
            lError = VmbErrorBadParameter;
    }

    if( VmbErrorSuccess == lError )
    {
        lError = lFeature->RunCommand();
    }

    return lError;
}

//
// Called when any failure occurs within the example.
// Ensures to stop streaming, close interface, close camera and shutdown Vimba
//
void cActionCommands::FailureShutdown()
{
    VmbErrorType lError = VmbErrorSuccess;

    // stop streaming
    if( true == this->mStreamingFlag )
    {
        lError = this->mCamera->StopContinuousImageAcquisition();
        if( VmbErrorSuccess != lError )
        {
            std::cout << "[F]...Could not stop camera acquisition. Reason: " << lError << std::endl;
        }

        std::cout << "......Streaming has been stopped." << std::endl;
    }

    // close camera
    if( true == this->mCameraFlag )
    {
        lError = this->mCamera->Close();
        if( VmbErrorSuccess != lError )
        {
            std::cout << "[F]...Could not close camera. Reason: " << lError << std::endl;
        }

        std::cout << "......Camera has been closed." << std::endl;
    }

    // close interface
    if( true == this->mInterfaceFlag )
    {
        lError = this->mInterface->Close();
        if( VmbErrorSuccess != lError )
        {
            std::cout << "[F]...Could not close interface. Reason: " << lError << std::endl;
        }

        std::cout << "......Interface has been closed." << std::endl;
    }

    // shutdown Vimba
    if( true == this->mSystemFlag )
    {
        lError = this->mSystem.Shutdown();
        if( VmbErrorSuccess != lError )
        {
            std::cout << "[F]...Could not shutdown Vimba. Reason: " << lError << std::endl;
        }

        std::cout << "......Vimba has been stooped." << std::endl;
    }
}

//
// Convert given string to IP address,
// using respective socket library (Winsock/Arpa)
//
// Parameters:
//  [in]    aString     String to be converted
//  [out]   aIPAddress  Decimal representation of given IP address string
//
VmbErrorType cActionCommands::ConvertStringToIPAddress( std::string aString, VmbUint32_t* aIPAddress )
{
    VmbErrorType lError = VmbErrorSuccess;
    VmbUint32_t lIP     = 0;

    // check parameter
    if( (true == aString.empty()) || (NULL == aIPAddress) )
    {
        std::cout << "[F]...Invalid parameter given" << std::endl;
        return VmbErrorBadParameter;
    }

    // convert given string to IP struct
    lIP = inet_addr( aString.c_str() );
    if( -1 == lIP )
    {
        lError = VmbErrorInvalidValue;
    }
    else
    {
        *aIPAddress = lIP;
    }

    return lError;
}

//
// Start Vimba and open camera with given string
//
// Parameters:
//  [in]    aCamera     The ID or IP address of the camera to work with
//
VmbErrorType cActionCommands::PrepareCamera( std::string aCamera )
{
    VmbErrorType lError = VmbErrorSuccess;

    // check parameter
    if( true == aCamera.empty() )
    {
        std::cout << "[F]...Invalid device ID or IP address given." << std::endl;
        return VmbErrorBadParameter;
    }

    // start Vimba
    lError = this->mSystem.Startup();
    if( VmbErrorSuccess != lError )
    {
        std::cout << "[F]...Could not start Vimba API. Reason: " << lError << std::endl;
        return lError;
    }

    this->mSystemFlag = true;
    std::cout << "......Vimba has been started." << std::endl;

    // open camera with given string (could be device ID or IP address)
    lError = this->mSystem.OpenCameraByID( aCamera.c_str(), VmbAccessModeFull, this->mCamera );
    if( VmbErrorSuccess != lError )
    {
        std::cout << "[F]...Could not open camera " << aCamera << ". Reason: " << lError << std::endl;
        this->FailureShutdown();
        return lError;
    }

    this->mCameraFlag = true;
    std::cout << "......Camera has been opened (" << aCamera << ")." << std::endl;

    return lError;
}

//
// Close camera and shutdown Vimba
//
VmbErrorType cActionCommands::StopCamera()
{
    VmbErrorType lError = VmbErrorSuccess;

    if( true == this->mCameraFlag )
    {
        // close camera
        lError = this->mCamera->Close();
        if( VmbErrorSuccess != lError )
        {
            std::cout << "[F]...Could not close camera. Reason: " << lError << std::endl;
        }

        this->mCameraFlag = false;
        std::cout << "......Camera has been closed." << std::endl;
    }

    if( true == this->mSystemFlag )
    {
        // shutdown Vimba
        lError = this->mSystem.Shutdown();
        if( VmbErrorSuccess != lError )
        {
            std::cout << "[F]...Could not stop Vimba. Reason: " << lError << std::endl;
        }

        this->mSystemFlag = false;
        std::cout << "......Vimba has been stopped." << std::endl;
    }

    return lError;
}

//
// Prepare trigger settings for given camera
//
VmbErrorType cActionCommands::PrepareTrigger()
{
    VmbErrorType    lError  = VmbErrorSuccess;
    FeaturePtr      lFeature;
    char*           lTemp   = "";;

    // select FrameStart trigger via TriggerSelector feature
    lTemp = "FrameStart";
    lError = this->SetFeatureValue( "TriggerSelector", eFeatureOwnerCamera, VmbFeatureDataEnum, lTemp, sizeof(lTemp) );
    if( VmbErrorSuccess != lError )
    {
        std::cout << "[F]...Could not set TriggerSelector to FrameStart. Reason: " << lError << std::endl;
        FailureShutdown();
        return lError;
    }

    // set trigger source to Action0
    lTemp = "Action0";
    lError = this->SetFeatureValue( "TriggerSource", eFeatureOwnerCamera, VmbFeatureDataEnum, lTemp, sizeof(lTemp) );
    if( VmbErrorSuccess != lError )
    {
        std::cout << "[F]...Could not set TriggerSource to 'Action0'. Reason: " << lError << ". Probably this camera does not support Action Commands." << std::endl;
        FailureShutdown();
        return lError;
    }

    // enable trigger
    lTemp = "On";
    lError = this->SetFeatureValue( "TriggerMode", eFeatureOwnerCamera, VmbFeatureDataEnum, lTemp, sizeof(lTemp) );
    if( VmbErrorSuccess != lError )
    {
        std::cout <<  "[F]...Could not enable TriggerMode for FrameStart. Reason: " << lError << std::endl;
        FailureShutdown();
        return lError;
    }

    std::cout << "......Trigger FrameStart has been activated and set to Action0" << std::endl;

    return lError;
}

//
// Set Action Command information to given feature owner.
// This could be Vimba system, interface or the camera
//
// Parameters:
//  [in]    aOwner      Feature owner, like System, Interface or Camera
//  [in]    aCommand    Action Command struct (device key, group key, group mask)
//
VmbErrorType cActionCommands::PrepareActionCommand( tFeatureOwner aOwner, tActionCommand* aCommand )
{
    VmbErrorType lError = VmbErrorSuccess;

    // check parameter
    if( NULL == aCommand )
    {
        std::cout << "[F]...Invalid Action Command given." << std::endl;
        FailureShutdown();
        return VmbErrorBadParameter;
    }

    // set device key
    lError = this->SetFeatureValue( "ActionDeviceKey", aOwner, VmbFeatureDataInt, (void*)(aCommand->mDeviceKey), sizeof(aCommand->mDeviceKey) );
    if( VmbErrorSuccess != lError )
    {
        std::cout << "[F]...Could not set ActionDeviceKey. Reason: " << lError << std::endl;
        FailureShutdown();
        return lError;
    }

    // set group key
    lError = this->SetFeatureValue( "ActionGroupKey", aOwner, VmbFeatureDataInt, (void*)(aCommand->mGroupKey), sizeof(aCommand->mGroupKey) );
    if( VmbErrorSuccess != lError )
    {
        std::cout << "[F]...Could not set ActionGroupKey. Reason: " << lError << std::endl;
        FailureShutdown();
        return lError;
    }

    // set group mask
    lError = this->SetFeatureValue( "ActionGroupMask", aOwner, VmbFeatureDataInt, (void*)(aCommand->mGroupMask), sizeof(aCommand->mGroupMask) );
    if( VmbErrorSuccess != lError )
    {
        std::cout << "[F]...Could not set ActionGroupMask. Reason: " << lError << std::endl;
        FailureShutdown();
        return lError;
    }

    std::cout << "......Action Command has been set (" << aCommand->mDeviceKey << ", " << aCommand->mGroupKey << ", " << aCommand->mGroupMask << ")" << std::endl;

    return lError;
}

//
// Prepare streaming settings in Vimba and the camera,
// like allocating the buffers, start capture engine, etc.
//
VmbErrorType cActionCommands::PrepareStreaming()
{
    VmbErrorType    lError = VmbErrorSuccess;
    FeaturePtr      lFeature;

    // set GVSP packet size to max value (MTU)
    // and wait until command is done
    lError = this->mCamera->GetFeatureByName( "GVSPAdjustPacketSize", lFeature );
    if( VmbErrorSuccess == lError )
    {
        lError = lFeature->RunCommand();
    }

    // check if any failure occurred
    if( VmbErrorSuccess != lError )
    {
        std::cout << "[F]...Could not set GVSP packet size. Reason: " << lError << std::endl;
        FailureShutdown();
        return lError;
    }

    // check if operation is done
    bool lFlag = false;
    do
    {
        lError = lFeature->IsCommandDone( lFlag );
        if( VmbErrorSuccess != lError )
        {
            break;
        }

    } while( VmbBoolFalse == lFlag );
    
    // get GVSP packet size, which was actually set in the camera
    VmbInt64_t lGVSPSize = 0;
    lError = this->GetFeatureValue( "GVSPPacketSize", eFeatureOwnerCamera, NULL, &lGVSPSize, NULL );
    if( VmbErrorSuccess != lError )
    {
        std::cout << "[F]...Could not get GVSP packet size. Reason: " << lError << std::endl;
    }

    std::cout << "......GVSP packet size has been set to maximum (" << lGVSPSize << ")" << std::endl;

    // create new frame observer
    this->mFrameObserver = new cFrameObserver( this->mCamera );

    // start continuous image acquisition
    lError = this->mCamera->StartContinuousImageAcquisition( NUM_FRAMES, IFrameObserverPtr(this->mFrameObserver) );
    if( VmbErrorSuccess != lError )
    {
        FailureShutdown();
        std::cout << "[F]...Could not start continuous image acquisition. Reason: " << lError << std::endl;
        return lError;
    }

    this->mStreamingFlag = true;
    std::cout << "......Camera acquisition has been started." << std::endl;

    return lError;
}

//
// End streaming
//
VmbErrorType cActionCommands::StopStreaming()
{
    VmbErrorType lError = VmbErrorSuccess;

    // stop continuous image acquisition
    lError = this->mCamera->StopContinuousImageAcquisition();
    if( VmbErrorSuccess != lError )
    {
        FailureShutdown();
        std::cout << "[F]...Could not stop streaming. Reason: " << lError << std::endl;
        return lError;
    }

    this->mStreamingFlag = false;
    std::cout << "......Camera acquisition has been stopped." << std::endl;

    return lError;
}

//
// Send Action Command  on system level.
// This command will be broadcasted on all network interfaces.
//
// Parameters:
//  [in]    aCamera     The ID or IP address of the camera to work with
//  [in]    aCommand    Action Command to be used by Vimba and camera
//
VmbErrorType cActionCommands::SendActionCommandOnAllInterfaces( std::string aCamera, tActionCommand aCommand )
{
    VmbErrorType lError = VmbErrorSuccess;

    // -start Vimba
    // -open camera in full access mode and get handle
    lError = PrepareCamera( aCamera );
    if( VmbErrorSuccess != lError )
    {
        return lError;
    }

    // -select FrameStart trigger feature
    // -set source to Action0
    // -enable trigger
    lError = PrepareTrigger();
    if( VmbErrorSuccess != lError )
    {
        return lError;
    }

    // Set Action Command to camera
    // -set device key
    // -set group key
    // -set group mask
    lError = PrepareActionCommand( eFeatureOwnerCamera, &aCommand );
    if( VmbErrorSuccess != lError )
    {
        return lError;
    }

    // -adjust GVSP packet size
    // -get payload size
    // -allocate memory for frame buffers
    // -announce frames and move them to buffer input pool
    // -start capture engine
    // -move frames to capture queue (buffer output queue)
    // -call start acquisition feature in the camera
    lError = PrepareStreaming();
    if( VmbErrorSuccess != lError )
    {
        return lError;
    }

    // determine if Action Command shall be send as uni- or broadcast
    // if IP address was given, send it as unicast
    VmbUint32_t lIP = 0;
    lError = ConvertStringToIPAddress( aCamera, &lIP );
    if( VmbErrorSuccess == lError )
    {
        // set IP address to Vimba
        lError = this->SetFeatureValue( "GevActionDestinationIPAddress", eFeatureOwnerSystem, VmbFeatureDataInt, &lIP, sizeof(lIP) );
        if( VmbErrorSuccess != lError )
        {
            std::cout << "[F]...Could not set IP address '" << aCamera << "' to Vimba. Reason: " << lError << std::endl;
        }

        std::cout << "......Action Command will be send as unicast to IP '" << aCamera << "' (" << lIP << ")" << std::endl;
    }

    #ifdef _WIN32
        std::cout << "\n<< Please hit 'a' to send prepared Action Command. To stop example hit 'q' >>\n\n";
    #else
        std::cout << "\n<< Please enter 'a' and return to send prepared Action Command. To stop example enter 'q' and return >>\n\n";
    #endif

    // repeat this until user hits ESC
    int lKey = 0;
    do
    {
        // wait for user input
        #ifdef _WIN32
            lKey = _getch();
        #else
            lKey = getchar();
        #endif

        if( 97 == lKey )
        {
            // set Action Command to Vimba system
            // -device key
            // -group key
            // -group mask
            lError = PrepareActionCommand( eFeatureOwnerSystem, &aCommand );
            if( VmbErrorSuccess == lError )
            {
                // send Action Command by calling command feature
                lError = RunCommand( "ActionCommand", eFeatureOwnerSystem );
                if( VmbErrorSuccess != lError )
                {
                    std::cout << "[F]...Could not send Action Command. Reason: " << lError << std::endl;
                    FailureShutdown();
                    return lError;
                }

                std::cout << "......Action Command has been sent." << std::endl;
            }
            else
            {
                return lError;
            }
        }

    } while( 113 != lKey );

    // stop streaming
    lError = StopStreaming();
    if( VmbErrorSuccess != lError )
    {
        return lError;
    }

    // -close camera
    // -shutdown Vimba
    lError = StopCamera();

    return lError;
}

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
VmbErrorType cActionCommands::SendActionCommandOnInterface( std::string aCamera, std::string aInterface, tActionCommand aCommand )
{
    VmbErrorType lError = VmbErrorSuccess;

    // -start Vimba
    // -open camera in full access mode and get handle
    lError = PrepareCamera( aCamera );
    if( VmbErrorSuccess != lError )
    {
        return lError;
    }

    // -select FrameStart trigger feature
    // -set source to Action0
    // -enable trigger
    lError = PrepareTrigger();
    if( VmbErrorSuccess != lError )
    {
        return lError;
    }

    // Set Action Command to camera
    // -set device key
    // -set group key
    // -set group mask
    lError = PrepareActionCommand( eFeatureOwnerCamera, &aCommand );
    if( VmbErrorSuccess != lError )
    {
        return lError;
    }

    // -adjust GVSP packet size
    // -get payload size
    // -allocate memory for frame buffers
    // -announce frames and move them to buffer input pool
    // -start capture engine
    // -move frames to capture queue (buffer output queue)
    // -call start acquisition feature in the camera
    lError = PrepareStreaming();
    if( VmbErrorSuccess != lError )
    {
        return lError;
    }

    // get available interfaces
    InterfacePtrVector lInterfaces;
    lError = this->mSystem.GetInterfaces( lInterfaces );
    if( (VmbErrorSuccess != lError) || (0 == lInterfaces.size()) )
    {
        std::cout << "[F]...Could not retrieve interfaces" << std::endl;
        FailureShutdown();
        return lError;
    }

    // print interface list
    bool lFound = false;
    int lIndex = 0;
    for( int i=0; i<lInterfaces.size(); ++i )
    {
        InterfacePtr lInterface = lInterfaces.at(i);
        std::string lInterfaceID = "";
        lError = lInterface->GetID( lInterfaceID );
        if( VmbErrorSuccess == lError )
        {
            std::cout << ".........[" << i << "] " << lInterfaceID << std::endl;

            // compare given interface ID with current one
            if( 0 == lInterfaceID.compare( aInterface ) )
            {
                // if interface ID matches, keep index
                lFound = true;
                lIndex = i;
            }
        }
    }

    // if no interface with given ID was found, return
    if( false == lFound )
    {
        std::cout << "[F]...Given interface with ID '" << aInterface << "' was not found!" << std::endl;
        FailureShutdown( );
        return VmbErrorBadParameter;
    }

    // get interface pointer
    this->mInterface = lInterfaces.at( lIndex );
    if( true == SP_ISNULL(this->mInterface) )
    {
        std::cout << "[F]...No valid interface pointer with given index found" << std::endl;
        FailureShutdown();
        return VmbErrorBadParameter;
    }

    // check interface type
    VmbInterfaceType lInterfaceType = VmbInterfaceUnknown;
    lError = this->mInterface->GetType( lInterfaceType );
    if( (VmbErrorSuccess != lError) || (VmbInterfaceEthernet != lInterfaceType) )
    {
        printf( "[F]...Selected interface is non-GigE interface!\n" );
        FailureShutdown();
        return VmbErrorBadParameter;
    }

    // open interface
    lError = this->mInterface->Open();
    if( VmbErrorSuccess != lError )
    {
        std::cout << "[F]...Could not open interface" << std::endl;
        FailureShutdown();
        return lError;
    }

    this->mInterfaceFlag = true;
    std::cout << "......Interface has been opened." << std::endl;

    // determine if Action Command shall be send as uni- or broadcast
    // if IP address was given, send it as unicast
    VmbUint32_t lIP = 0;
    lError = ConvertStringToIPAddress( aCamera, &lIP );
    if( VmbErrorSuccess == lError )
    {
        // set IP address to Vimba
        lError = this->SetFeatureValue( "GevActionDestinationIPAddress", eFeatureOwnerSystem, VmbFeatureDataInt, &lIP, sizeof(lIP) );
        if( VmbErrorSuccess != lError )
        {
            std::cout << "[F]...Could not set IP address '" << aCamera << "' to Vimba. Reason: " << lError << std::endl;
        }

        std::cout << "......Action Command will be send as unicast to IP '" << aCamera << "' (" << lIP << ")" << std::endl;
    }

    #ifdef _WIN32
        std::cout << "\n<< Please hit 'a' to send prepared Action Command. To stop example hit 'q' >>\n\n";
    #else
        std::cout << "\n<< Please enter 'a' and return to send prepared Action Command. To stop example enter 'q' and return >>\n\n";
    #endif

    // repeat this until user hits ESC
    int lKey = 0;
    do
    {
        // wait for user input
        #ifdef _WIN32
            lKey = _getch();
        #else
            lKey = getchar();
        #endif

        if( 97 == lKey )
        {
            // set Action Command to Vimba system
            // -device key
            // -group key
            // -group mask
            lError = PrepareActionCommand( eFeatureOwnerInterface, &aCommand );
            if( VmbErrorSuccess == lError )
            {
                // send Action Command by calling command feature
                lError = RunCommand( "ActionCommand", eFeatureOwnerInterface );
                if( VmbErrorSuccess != lError )
                {
                    std::cout << "[F]...Could not send Action Command. Reason: " << lError << std::endl;
                    FailureShutdown();
                    return lError;
                }

                std::cout << "......Action Command has been sent." << std::endl;
            }
            else
            {
                return lError;
            }
        }

    } while( 113 != lKey );

    // close interface
    lError = this->mInterface->Close();
    if( VmbErrorSuccess != lError )
    {
        std::cout << "Could not close interface. Reason: " << lError << std::endl;
    }

    this->mInterfaceFlag = false;
    std::cout << "......Interface has been closed." << std::endl;

    // stop streaming
    lError = StopStreaming();
    if( VmbErrorSuccess != lError )
    {
        return lError;
    }

    // -close camera
    // -shutdown Vimba
    lError = StopCamera();

    return lError;
}

}}} // namespace AVT::VmbAPI::Examples
