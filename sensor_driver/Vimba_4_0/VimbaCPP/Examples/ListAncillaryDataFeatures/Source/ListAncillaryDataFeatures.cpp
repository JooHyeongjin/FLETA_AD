/*=============================================================================
  Copyright (C) 2014 - 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:         ListAncillaryDataFeatures.cpp

  Description:  The ListAncillaryDataFeatures example will list all available
                features of a camera that are found by VimbaCPP.

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
#include <ListAncillaryDataFeatures.h>

#include "VimbaCPP/Include/VimbaCPP.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

// Purpose:     Uses an API feature to set the biggest packet size possible
//
// Parameter:
// [in ]        CamerPtr pCamera            The camera to work on
VmbErrorType AdjustPacketSize(CameraPtr pCamera)
{
    FeaturePtr      pPacketSize;
    VmbErrorType    err;
    err = SP_ACCESS( pCamera )->GetFeatureByName( "GVSPAdjustPacketSize", pPacketSize );
    if( VmbErrorSuccess ==  err )
    {
        err = SP_ACCESS( pPacketSize )->RunCommand() ;
        if( VmbErrorSuccess == err)
        {
            bool bIsCommandDone = false;
            do
            {
                if( VmbErrorSuccess != SP_ACCESS( pPacketSize )->IsCommandDone( bIsCommandDone ) )
                {
                    break;
                }
            } while( false == bIsCommandDone );
        }
    }
    return err;
}

enum 
{
    ACQ_TIMEOUT = 1000,
};

// Purpose:     Prints out the value of a given feature
//
// Parameter:
// [in ]        const FeaturePtr& feature       A reference to the feature shared pointer
void PrintFeatureValue( const FeaturePtr& pFeature )
{
    VmbFeatureDataType  featureType;
    VmbErrorType        err             = pFeature->GetDataType( featureType );
    
    if( VmbErrorSuccess != err )
    {
        std::cout << "[Could not get feature Data Type. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]\n";
    }
    else
    {
        std::cout << "/// Value          : ";
        switch( featureType )
        {
        case VmbFeatureDataBool:
            {
                VmbBool_t value;
                err = pFeature->GetValue( value );
                if ( VmbErrorSuccess == err )
                {
                    std::cout << value << "\n";
                }
            }
            break;
        case VmbFeatureDataEnum:
            {
                std::string value;
                err = pFeature->GetValue( value );
                if ( VmbErrorSuccess == err )
                {
                    std::cout << value << "\n";
                }
            }
            break;
        case VmbFeatureDataFloat:
            {
                double value;
                err = pFeature->GetValue( value );
                if( VmbErrorSuccess == err)
                {
                    std::cout << value << "\n";
                }
            }
            break;
        case VmbFeatureDataInt:
            {
                VmbInt64_t value;
                err = pFeature->GetValue( value );
                if( VmbErrorSuccess == err)
                {
                    std::cout << value << "\n";
                }
            }
            break;
        case VmbFeatureDataString:
            {
                std::string value;
                err = pFeature->GetValue( value );
                if( VmbErrorSuccess == err)
                {
                    std::cout << value << "\n";
                }
            }
            break;
        case VmbFeatureDataCommand:
        default:
            std::cout << "[None]" << "\n";
            break;
        }
        
        if( VmbErrorSuccess == err )
        {
            std::cout << "\n";
        }
        else
        {
            std::cout << "Could not get feature value. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n\n";
        }
    }
}

// Purpose:     Prints out all details of a given feature
//
// Parameter:
// [in ]        const FeaturePtr& pFeature       A reference to the feature shared pointer
void PrintFeature( const FeaturePtr& pFeature )
{
    std::string name;                                                    // The name of the feature
    std::string displayName;                                             // The display name of the feature
    std::string toolTip;                                                 // A short description of the feature
    std::string description;                                             // A long description of the feature
    std::string category;                                                // A category to group features
    std::string sfncNamespace;                                           // The Standard Feature Naming Convention namespace
    std::string unit;                                                    // The measurement unit of the value

    std::ostringstream ErrorStream;

    VmbErrorType err = pFeature->GetName( name );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Name. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        name = ErrorStream.str();
    }

    err = pFeature->GetDisplayName( displayName );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Display Name. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        displayName = ErrorStream.str();
    }

    err = pFeature->GetToolTip( toolTip );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Tooltip. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        toolTip = ErrorStream.str();
    }

    err = pFeature->GetDescription( description );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Description. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        description = ErrorStream.str();
    }

    err = pFeature->GetCategory( category );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Category. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        category = ErrorStream.str();
    }

    err = pFeature->GetSFNCNamespace( sfncNamespace );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature SNFC Namespace. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        sfncNamespace = ErrorStream.str();
    }

    err = pFeature->GetUnit( unit );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Unit. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        unit = ErrorStream.str();
    }

    std::cout << "/// Feature Name   : " << name             << "\n";
    std::cout << "/// Display Name   : " << displayName      << "\n";
    std::cout << "/// Tooltip        : " << toolTip          << "\n";
    std::cout << "/// Description    : " << description      << "\n";
    std::cout << "/// SNFC Namespace : " << sfncNamespace    << "\n";
    
    PrintFeatureValue( pFeature );
}

// Purpose:     Pints out features from the ancillary data. Ancillary data is part of a frame,
//              therefore we need to capture a single frame beforehand.
//              If no camera ID string was passed we use the first camera found.
//
// Parameter:
// [in ]        string cameraID     The ID of the camera to use
void ListAncillaryDataFeatures::Print( std::string cameraID )
{
    VimbaSystem&        sys         = VimbaSystem::GetInstance();                       // Get a reference to the VimbaSystem singleton
    std::cout << "Vimba C++ API Version " << sys << "\n";                                      // Print out version of Vimba
    VmbErrorType        err         = sys.Startup();                                    // Initialize the Vimba API
    FeaturePtrVector    features;                                                       // A vector of std::shared_ptr<AVT::VmbAPI::Feature> objects
    CameraPtr           pCamera     = CameraPtr();                                      // Our camera

    std::stringstream errorString;

    if( VmbErrorSuccess == err )
    {
        if( cameraID.empty() )                                                          // If no ID was provided use the first camera
        {
            CameraPtrVector cameras;
            err = sys.GetCameras( cameras );
            if(     VmbErrorSuccess == err
                &&  !cameras.empty() )
            {
                pCamera = cameras[0];                                                   // Get the camera
                err = pCamera->Open( VmbAccessModeFull );                               // Open the camera
                if( VmbErrorSuccess == err )
                {
                    err = pCamera->GetID( cameraID );
                }
            }
        }
        else
        {
            err = sys.OpenCameraByID(   cameraID.c_str(),                               // Get and open the camera
                                        VmbAccessModeFull,
                                        pCamera ); 
        }

        if( !SP_ISNULL( pCamera ))
        {
            if ( VmbErrorSuccess == err )
            {
                AdjustPacketSize( pCamera );
                std::cout << "Printing all ancillary data features of camera with ID: " << cameraID << "\n\n";
            
                FeaturePtr pFeature;
                err = pCamera->GetFeatureByName( "ChunkModeActive", pFeature );
                if( VmbErrorSuccess == err )
                {
                    err = pFeature->SetValue( true );                                       // Enable ancillary data
                    if( VmbErrorSuccess == err )
                    {
                        std::cout << "Capture a single frame\n\n";                          // In order to fill the ancillary data we need to fill a frame
                        
                        FramePtr pFrame;
                        err = pCamera->AcquireSingleImage( pFrame, ACQ_TIMEOUT );
                        if ( VmbErrorSuccess == err )
                        {
                            VmbFrameStatusType status;
                            err = pFrame->GetReceiveStatus( status );                       // Check whether we received a complete frame
                            if( VmbErrorSuccess == err )
                            {
                                if ( VmbFrameStatusComplete == status )
                                {
                                    AncillaryDataPtr pAncillaryData;
                                    err = pFrame->GetAncillaryData( pAncillaryData );       // Get the ancillary data of the frame
                                    if ( VmbErrorSuccess == err )
                                    {
                                        err = pAncillaryData->Open();
                                        if( VmbErrorSuccess == err)
                                        {
                                            err = pAncillaryData->GetFeatures( features );  // Fetch all features of the ancillary data
                                            if( VmbErrorSuccess == err )
                                            {
                                                // Query all static details as well as the value of all fetched features and print them out.
                                                std::for_each( features.begin(), features.end(), PrintFeature );
                                            }
                                            else
                                            {
                                                std::cout << "Could not get features. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                                            }

                                            pAncillaryData->Close();
                                        }
                                        else
                                        {
                                            std::cout << "Could not open ancillary data. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                                        }
                                    }
                                    else
                                    {
                                        std::cout << "Could not get ancillary data. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                                    }
                                }
                                else if( VmbFrameStatusIncomplete == status )
                                {
                                    std::cout << "Could not acquire complete frame. Receive status: " << err << "\n";
                                }
                                else
                                {
                                    std::cout << "Could not acquire frame. Receive status: " << err << "\n";
                                }
                            }
                            else
                            {
                                std::cout << "Could not get frame receive status. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                            }
                        }
                        else
                        {
                            std::cout << "Could not acquire image. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                        }
                    }
                    else
                    {
                        std::cout << "Could not enable ancillary data. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                    }
                }
                else if( VmbErrorNotFound == err )
                {
                    std::cout << "The camera does not provide ancillary data. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                }
                else
                {
                    std::cout << "Could not query for the presence of ancillary data. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
                }
            

                pCamera->Close();
            }
            else
            {
                std::cout << "Could not open camera. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
            }
        }
        else
        {
            std::cout << "No camera available.\n";
        }

        sys.Shutdown();                                                                 // Finally close Vimba
    }
    else
    {
        std::cout << "Could not start system. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
    }
}

}}} // namespace AVT::VmbAPI::Examples
