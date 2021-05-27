/*=============================================================================
  Copyright (C) 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        program.cpp

  Description: LoadSaveSettings example of VimbaCPP.
               

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
#include <sstream>
#include <VimbaCPP/Include/VimbaCPP.h>

int main( int argc, char* argv[] )
{
    using namespace AVT::VmbAPI;

    VmbErrorType err = VmbErrorSuccess;
    std::stringstream ss;
    bool apiFlag = false;
    bool cameraFlag = false;

    std::cout << std::endl;
    std::cout << "////////////////////////////////////////////" << std::endl;
    std::cout << "/// Vimba API Load/Save Settings Example ///" << std::endl;
    std::cout << "////////////////////////////////////////////" << std::endl;
    std::cout << std::endl;
    
//  create camera pointer
//  get VimbaCPP instance singleton
    CameraPtr pCam;
    VimbaSystem &sys = VimbaSystem::GetInstance();

    try
    {
    //  start Vimba API
        err = sys.Startup();
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not start Vimba [error code: " << err << "]";
            std::cout << ss.str() << std::endl;
            throw std::exception();
        }

        apiFlag = true;
        std::cout << "--> VimbaCPP has been started" << std::endl;

    //  get connected cameras from VimbaCPP
        CameraPtrVector pCameras;
        err = sys.GetCameras( pCameras );
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not get connected cameras [error code: " << err << "]";
            std::cout << ss.str() << std::endl;

            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            throw std::exception();
        }

    //  select first camera in list
        pCam = pCameras[0];

    //  open camera
        err = pCam->Open( VmbAccessModeFull );
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not open camera [error code: " << err << "]";
            std::cout << ss.str() << std::endl;
            
            err = pCam->Close();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not close camera [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }

            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }

            throw std::exception();
        }

    //  get camera id
        std::string cameraId;
        err = pCam->GetID( cameraId );
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not get camera id [error code: " << err << "]";
            std::cout << ss.str() << std::endl;

            err = pCam->Close();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not close camera [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }

            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }

            throw std::exception();
        }

        cameraFlag = true;
        ss.str( "" );
        ss << "--> Camera with id '" << cameraId << "' has been opened";
        std::cout << ss.str() << std::endl;

    //  create xml file name
        ss.str( "" );
        ss << cameraId << ".xml";
        std::string xmlFile = ss.str();

    //  -------------------------------------------------------------------------------------
    //  setup load/save settings behaviour:
    //      there are three different ways in VimbaCPP to setup the behaviour for
    //      loading and saving feature values with load/save settings implementation.
    //      SaveCameraSettings() and LoadCameraSettings() can be called either with
    //      an created struct of type 'VmbFeaturePersistSettings_t' or not. The third
    //      alternative is to call LoadSaveSettingsSetup() beforehand and provide the
    //      same parameters as it will be done by the struct.
    //
    //      (1) default usage:
    //          pCam->SaveCameraSettings( xmlFile );
    //          pCam->LoadCameraSettings( xmlFile );
    //
    //      (2) usage with settings struct:
    //          VmbFeaturePersistSettings_t settingsStruct;
    //          settingsStruct.loggingLevel = 4;                        //  set logging level (0:info only, 1: with errors, 2: with warnings, 3: with debug, 4: with traces)
    //          settingsStruct.maxIterations = 5;                       //  since its difficult to catch all feature dependencies during loading multiple
                                                                        //  iterations are used (compare desired value with camera value and write it to camera)
    //          settingsStruct.persistType = VmbFeaturePersistNoLUT;    //  set which features shall be persisted (saved to XML):
                                                                        //  VmbFeaturePersistAll: all features shall be persisted (including LUTs).
                                                                        //  VmbFeaturePersistStreamable: only streamable features shall be persisted.
                                                                        //  VmbFeaturePersistNoLUT: all features shall be persisted except for LUT,
                                                                        //  which is the recommended setting, because it might be very time consuming.
    //          pCam->SaveCameraSettings( xmlFile, &settingsStruct );
    //          pCam->LoadCameraSettings( xmlFile, &settingsStruct );
    //
    //      (3) usage with setup method:
    //          pCam->LoadSaveSettingsSetup( VmbFeaturePersistNoLUT, 5, 4 );
    //          pCam->SaveCameraSettings( xmlFile );
    //          pCam->LoadCameraSettings( xmlFile );
    //  -------------------------------------------------------------------------------------

    //  call VimbaCPP method for saving all feature values
        err = pCam->SaveCameraSettings( xmlFile );
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not save camera settings to file '" << xmlFile << "' [error code: " << err << "]";
            std::cout << ss.str() << std::endl;

            err = pCam->Close();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not close camera [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            cameraFlag = false;

            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            apiFlag = false;

            throw std::exception();
        }

        ss.str( "" );
        ss << "--> Feature values have been saved to '" << xmlFile << "'";
        std::cout << ss.str() << std::endl;

    //  get feature selector for user set
        FeaturePtr feature;
        err = pCam->GetFeatureByName( "UserSetSelector", feature );
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not get feature 'UserSetSelector' [error code: " << err << "]";
            std::cout << ss.str() << std::endl;

            err = pCam->Close();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not close camera [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            cameraFlag = false;

            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            apiFlag = false;

            throw std::exception();
        }

    //  set value of selector to 'Default'
        err = feature->SetValue( "Default" );
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not set value of feature 'UserSetSelector' [error code: " << err << "]";
            std::cout << ss.str() << std::endl;

            err = pCam->Close();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not close camera [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            cameraFlag = false;

            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            apiFlag = false;

            throw std::exception();
        }

    //  get feature command 'UserSetLoad'
        err = pCam->GetFeatureByName( "UserSetLoad", feature );
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not get feature 'UserSetLoad' [error code: " << err << "]";
            std::cout << ss.str() << std::endl;

            err = pCam->Close();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not close camera [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            cameraFlag = false;

            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            apiFlag = false;

            throw std::exception();
        }

    //  load selected user set
        err = feature->RunCommand();
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not run command 'UserSetLoad' [error code: " << err << "]";
            std::cout << ss.str() << std::endl;

            err = pCam->Close();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not close camera [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            cameraFlag = false;

            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            apiFlag = false;

            throw std::exception();
        }

        std::cout << "--> All feature values have been restored to default" << std::endl;

    //  create settings struct to determine behaviour during loading
        VmbFeaturePersistSettings_t settingsStruct;
        settingsStruct.loggingLevel = 4;
        settingsStruct.maxIterations = 5;
        settingsStruct.persistType = VmbFeaturePersistNoLUT;

    //  re-load saved settings from file
        err = pCam->LoadCameraSettings( xmlFile, &settingsStruct );
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not load camera settings to file '" << xmlFile << "' [error code: " << err << "]";
            std::cout << ss.str() << std::endl;

            err = pCam->Close();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not close camera [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            cameraFlag = false;

            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            apiFlag = false;

            throw std::exception();
        }

        ss.str( "" );
        ss << "--> Feature values have been loaded from given XML file '" << xmlFile << "'";
        std::cout << ss.str() << std::endl;

    //  close camera
        err = pCam->Close();
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not close camera [error code: " << err << "]";
            std::cout << ss.str() << std::endl;

            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
            apiFlag = false;

            throw std::exception();
        }
        cameraFlag = false;

        std::cout << "--> Camera has been closed" << std::endl;

    //  shutdown Vimba
        err = sys.Shutdown();
        if( VmbErrorSuccess != err )
        {
            ss.str( "" );
            ss << "Could not shutdown Vimba [error code: " << err << "]";
            std::cout << ss.str() << std::endl;
            throw std::exception();
        }
        apiFlag = false;

        std::cout << "--> VimbaCPP has been shut down" << std::endl;

    }
    catch( std::exception &e )
    {
        std::cout << "[Exception] " << e.what() << std::endl;
        
        if( true == cameraFlag )
        {
            err = pCam->Close();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not close camera [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
        }
        if( true == apiFlag )
        {
            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
        }

    }
    catch(...)
    {
        std::cout << "[Exception]" << std::endl;

        if( true == cameraFlag )
        {
            err = pCam->Close();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not close camera [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
        }
        if( true == apiFlag )
        {
            err = sys.Shutdown();
            if( VmbErrorSuccess != err )
            {
                ss.str( "" );
                ss << "Could not shutdown Vimba [error code: " << err << "]";
                std::cout << ss.str() << std::endl;
            }
        }

    }

    std::cout << std::endl << "<<press any key to close example>>" << std::endl;
    std::cin.get();
    return err;

}
