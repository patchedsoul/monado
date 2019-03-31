// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Space, space, space, SPAAAAAAAAAAAAAAAAAAAAAAAAAACE!
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup oxr_api
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xrt/xrt_compiler.h"

#include "util/u_debug.h"

#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_two_call.h"

#include "oxr_api_funcs.h"
#include "oxr_api_verify.h"


XrResult
oxr_xrCreateActionSpace(XrAction action,
                        const XrActionSpaceCreateInfo* createInfo,
                        XrSpace* space)
{
	struct oxr_action* act;
	struct oxr_logger log;
	OXR_VERIFY_ACTION_AND_INIT_LOG(&log, action, act,
	                               "xrCreateActionSpace");
	OXR_VERIFY_ARG_TYPE_AND_NULL(&log, createInfo,
	                             XR_TYPE_ACTION_SPACE_CREATE_INFO);

	//! @todo Implement
	return oxr_error(&log, XR_ERROR_HANDLE_INVALID, " not implemented");
}

static const XrReferenceSpaceType session_spaces[] = {
    XR_REFERENCE_SPACE_TYPE_VIEW,
    XR_REFERENCE_SPACE_TYPE_LOCAL,
    XR_REFERENCE_SPACE_TYPE_STAGE,
};

XrResult
oxr_xrEnumerateReferenceSpaces(XrSession session,
                               uint32_t spaceCapacityInput,
                               uint32_t* spaceCountOutput,
                               XrReferenceSpaceType* spaces)
{
	struct oxr_session* sess;
	struct oxr_logger log;
	OXR_VERIFY_SESSION_AND_INIT_LOG(&log, session, sess,
	                                "xrEnumerateReferenceSpaces");

	OXR_TWO_CALL_HELPER(&log, spaceCapacityInput, spaceCountOutput, spaces,
	                    ARRAY_SIZE(session_spaces), session_spaces);
}

XrResult
oxr_xrGetReferenceSpaceBoundsRect(XrSession session,
                                  XrReferenceSpaceType referenceSpaceType,
                                  XrExtent2Df* bounds)
{
	struct oxr_session* sess;
	struct oxr_logger log;
	OXR_VERIFY_SESSION_AND_INIT_LOG(&log, session, sess,
	                                "xrGetReferenceSpaceBoundsRect");

	//! @todo Implement
	return oxr_error(&log, XR_ERROR_FUNCTION_UNSUPPORTED,
	                 " not implemented");
}

XrResult
oxr_xrCreateReferenceSpace(XrSession session,
                           const XrReferenceSpaceCreateInfo* createInfo,
                           XrSpace* out_space)
{
	XrResult ret;
	struct oxr_session* sess;
	struct oxr_space* spc;
	struct oxr_logger log;
	OXR_VERIFY_SESSION_AND_INIT_LOG(&log, session, sess,
	                                "xrCreateReferenceSpace");
	OXR_VERIFY_ARG_TYPE_AND_NULL(&log, createInfo,
	                             XR_TYPE_REFERENCE_SPACE_CREATE_INFO);

	ret = oxr_space_reference_create(&log, sess, createInfo, &spc);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	*out_space = oxr_space_to_openxr(spc);

	return XR_SUCCESS;
}

XrResult
oxr_xrLocateSpace(XrSpace space,
                  XrSpace baseSpace,
                  XrTime time,
                  XrSpaceRelation* relation)
{
	struct oxr_space* spc;
	struct oxr_space* baseSpc;
	struct oxr_logger log;
	OXR_VERIFY_SPACE_AND_INIT_LOG(&log, space, spc, "xrLocateSpace");
	OXR_VERIFY_SPACE_NOT_NULL(&log, baseSpace, baseSpc);
	OXR_VERIFY_ARG_NOT_NULL(&log, relation);

	return oxr_space_locate(&log, spc, baseSpc, time, relation);
}

XrResult
oxr_xrDestroySpace(XrSpace space)
{
	struct oxr_space* spc;
	struct oxr_logger log;
	OXR_VERIFY_SPACE_AND_INIT_LOG(&log, space, spc, "xrDestroySpace");

	return oxr_space_destroy(&log, spc);
}
