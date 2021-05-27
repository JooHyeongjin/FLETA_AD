/*=============================================================================
  Copyright (C) 2012 - 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        BandwidthHelper.h

  Description: The BandwidthHelper example demonstrates how to get and set the
               bandwidth used by a camera using VimbaCPP.

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

#ifndef AVT_VMBAPI_HELPER_BANDWIDTHHELPER_H
#define AVT_VMBAPI_HELPER_BANDWIDTHHELPER_H

#include "VimbaCPP/Include/VimbaCPP.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

class BandwidthHelper
{
  public:
    //
    // Calculates the current bandwidth usage of a camera in relation to a free bus / network
    //
    // Parameters:
    //  [in]    pCamera             The camera to work on
    //  [out]   rfBandwidth         The current bandwidth usage (maximum 1)
    //
    // Returns:
    //  An API status code
    //
    static VmbErrorType GetBandwidthUsage( CameraPtr pCamera, double &bandwidth );

    //
    // Sets the current bandwidth usage in relation to a free bus / network
    //
    // Parameters:
    //  [in]    pCamera             The camera to work on
    //  [out]   fBandwidth          The bandwidth to be set (maximum 1)
    //
    // Returns:
    //  An API status code
    //
    static VmbErrorType SetBandwidthUsage( CameraPtr pCamera, double bandwidth );

    //
    // The relative minimum bandwidth usage as reported by the device
    //
    // Parameters:
    //  [in]    pCamera             The camera to work on
    //  [out    rfBandwidth         The ratio of minimum and maximum of either stream bytes per second or the packet size
    //
    // Returns:
    //  An API status code
    //
    static VmbErrorType GetMinPossibleBandwidthUsage( CameraPtr pCamera, double &bandwidth );

    //
    // The relative maximum bandwidth usage as reported by the device
    //
    // Parameters:
    //  [in]    pCamera             The camera to work on
    //  [out    rfBandwidth         The ratio of maximum packet size as reported by the device and the maximum of the bus (for technologies other than fire wire always 1)
    //
    // Returns:
    //  An API status code
    //
    static VmbErrorType GetMaxPossibleBandwidthUsage( CameraPtr pCamera, double &bandwidth );

    //
    // Converts the interface type enum to a string representation
    //
    // Parameters:
    //  [in]    interfaceType       The interface enum to convert
    //
    // Returns:
    //  The string representation of the given enum
    //
    static std::string InterfaceToString( VmbInterfaceType interfaceType );

  private:
    // No default ctor
    BandwidthHelper();
};

}}} // AVT:VmbAPI::Examples

#endif
