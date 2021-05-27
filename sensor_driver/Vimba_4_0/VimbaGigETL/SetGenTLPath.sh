#!/bin/sh
#==============================================================================
# Copyright (C) 2017 Allied Vision Technologies.  All Rights Reserved.
#
# Redistribution of this file, in original or modified form, without
# prior written consent of Allied Vision Technologies is prohibited.
#
#------------------------------------------------------------------------------
#
# File:			SetGenTLPath.sh
#
# Description:	This script sets the GENICAM_GENTL32_PATH and
#	       		GENICAM_GENTL64_PATH environment variable for the current shell
#	       		only.
#	       		This can become necessary when running a Vimba application under
#				a not logged-in user like a service.
#
# Note:			This script needs to be executed sourced. That is, using the
#				same shell the application will be launched in. This is done
#				by calling the script with the built-in shell command source
#				or . (period), e.g. . SetGenTLPath.sh
#
#------------------------------------------------------------------------------
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
# NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
# DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, 
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#==============================================================================
CWD=$(dirname $(readlink -f $_))
UNAME=$(uname -m)

if [ ${UNAME} = armv6l ] || [ ${UNAME} = armv7l ]
then
ARCH=arm
WORDSIZE=32
elif [ ${UNAME} = aarch64 ]
then
ARCH=arm
WORDSIZE=64
elif [ ${UNAME} = i386 ] || [ ${UNAME} = i486 ] || [ ${UNAME} = i586 ] || [ ${UNAME} = i686 ] || [ ${UNAME} = x86_32 ]
then
ARCH=x86
WORDSIZE=32
elif [ ${UNAME} = amd64 ] || [ ${UNAME} = x86_64 ]
then
ARCH=x86
WORDSIZE=64
else
   echo "
   Error: Incompatible system architecture found." 1>&2
   exit 1
fi

if [ "$_" = "$0" ]
then
  echo "
  Please asure to run this script using the 'source' shell command.
  This is done through: '. SetGenTLPath.sh'
  Please note the whitespace.
  Further help can be found by typing 'help source' or 'help .' in your shell.
  "
  exit 1
fi

if [ ${WORDSIZE} = 32 ] || [ ${WORDSIZE} = both ]
then
	TL_PATH_32BIT=$CWD/CTI/${ARCH}_32bit
	echo "
  Setting the GENICAM_GENTL32_PATH to $TL_PATH_32BIT for this shell only."
	export GENICAM_GENTL32_PATH=:$TL_PATH_32BIT
fi	

if [ ${WORDSIZE} = 64 ] || [ ${WORDSIZE} = both ]
then
	TL_PATH_64BIT=$CWD/CTI/${ARCH}_64bit
	echo "
  Setting the GENICAM_GENTL64_PATH to $TL_PATH_64BIT for this shell only."
	export GENICAM_GENTL64_PATH=:$TL_PATH_64BIT	
fi

echo "  Done
"
