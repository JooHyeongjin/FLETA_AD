/*=============================================================================
  Copyright (C) 2014 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:         ListAncillaryDataFeatures.h

  Description:  The ListAncillaryDataFeatures example will list all available
                features of the ancillary data.

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

#ifndef AVT_VMBAPI_EXAMPLES_LISTANCILLARYDATAFEATURES
#define AVT_VMBAPI_EXAMPLES_LISTANCILLARYDATAFEATURES

#include <string>

namespace AVT {
namespace VmbAPI {
namespace Examples {

class ListAncillaryDataFeatures
{
public:
    // Purpose:     Pints out features from the ancillary data. Ancillary data is part of a frame,
    //              therefore we need to capture a single frame beforehand.
    //              If no camera ID string was passed we use the first camera found.
    //
    // Parameter:
    // [in ]        string cameraID     The ID of the camera to use
    static void Print( std::string cameraID );
};

}}} // namespace AVT::VmbAPI::Examples

#endif
