/*=============================================================================
  Copyright (C) 2014 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        EventObserver.h

  Description: The EventObserver can be attached to a camera feature. On Feature
               changing the function FeatureChanged() is called.
  
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

#include "EventObserver.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

EventObserver::EventObserver( AVT::VmbAPI::CameraPtr pCam )
    :   m_pCam( pCam )
{
}

// Purpose: This function will be called when the observed feature is changing.
//
// Parameter:
// [in ]    const AVT::VmbAPI::FeaturePtr   &feature         changed feature
//
void EventObserver::FeatureChanged( const AVT::VmbAPI::FeaturePtr &feature )
{
    if( feature != NULL )
    {
        std::string featureName;
        if ( !feature->GetName( featureName ) )
        {
            std::cout << featureName << " (Event) has changed to ";

            AVT::VmbAPI::FeaturePtr pFeature;
            VmbInt64_t nID;
            if (    !( m_pCam->GetFeatureByName( featureName.c_str(), pFeature ) )
                &&  !( pFeature->GetValue( nID ) ) )
            {
                std::cout << nID << std::endl;
            }
        }
        else
        {
            std::cout << "An error occurred" << std::endl;
        }
    }
}

}}} // namespace AVT::VmbAPI::Examples