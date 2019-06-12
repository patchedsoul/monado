// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Action related API entrypoint functions.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup oxr_api
 */

#include <stdio.h>

#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_handle.h"

#include "util/u_debug.h"

#include "oxr_api_funcs.h"
#include "oxr_api_verify.h"


/*
 *
 * Session - action functions.
 *
 */

XrResult
oxr_xrSyncActionData(XrSession session,
                     uint32_t countActionSets,
                     const XrActiveActionSet* actionSets)
{
	struct oxr_session* sess;
	struct oxr_logger log;
	OXR_VERIFY_SESSION_AND_INIT_LOG(&log, session, sess,
	                                "xrSyncActionData");

	if (countActionSets == 0) {
		return oxr_error(&log, XR_ERROR_VALIDATION_FAILURE,
		                 "(countActionSets == 0)");
	}

	for (uint32_t i = 0; i < countActionSets; i++) {
		struct oxr_action_set* act_set = NULL;
		OXR_VERIFY_ARG_TYPE_AND_NULL(&log, (&actionSets[i]),
		                             XR_TYPE_ACTIVE_ACTION_SET);
		OXR_VERIFY_ACTIONSET_NOT_NULL(&log, actionSets[i].actionSet,
		                              act_set);

		oxr_verify_subaction_path_sync(&log, sess->sys->inst,
		                               actionSets[i].subactionPath, i);
	}

	return oxr_action_sync_data(&log, sess, countActionSets, actionSets);
}

XrResult
oxr_xrSetInteractionProfileSuggestedBindings(
    XrSession session,
    const XrInteractionProfileSuggestedBinding* suggestedBindings)
{
	struct oxr_session* sess;
	struct oxr_logger log;
	OXR_VERIFY_SESSION_AND_INIT_LOG(
	    &log, session, sess, "xrSetInteractionProfileSuggestedBindings");
	OXR_VERIFY_ARG_TYPE_AND_NULL(
	    &log, suggestedBindings,
	    XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING);

	for (size_t i = 0; i < suggestedBindings->countSuggestedBindings; i++) {
		const XrActionSuggestedBinding* s =
		    &suggestedBindings->suggestedBindings[i];

		struct oxr_action* dummy;
		OXR_VERIFY_ACTION_NOT_NULL(&log, s->action, dummy);

		//! @todo verify path (s->binding).
	}

	return oxr_action_set_interaction_profile_suggested_bindings(
	    &log, sess, suggestedBindings);
}

XrResult
oxr_xrGetCurrentInteractionProfile(XrSession session,
                                   XrPath topLevelUserPath,
                                   XrInteractionProfileInfo* interactionProfile)
{
	struct oxr_session* sess;
	struct oxr_logger log;
	OXR_VERIFY_SESSION_AND_INIT_LOG(&log, session, sess,
	                                "xrGetCurrentInteractionProfile");
	OXR_VERIFY_ARG_TYPE_AND_NULL(&log, interactionProfile,
	                             XR_TYPE_INTERACTION_PROFILE_INFO);

	return oxr_action_get_current_interaction_profile(
	    &log, sess, topLevelUserPath, interactionProfile);
}

XrResult
oxr_xrGetInputSourceLocalizedName(
    XrSession session,
    XrPath source,
    XrInputSourceLocalizedNameFlags whichComponents,
    uint32_t bufferCapacityInput,
    uint32_t* bufferCountOutput,
    char* buffer)
{
	struct oxr_session* sess;
	struct oxr_logger log;
	OXR_VERIFY_SESSION_AND_INIT_LOG(&log, session, sess,
	                                "xrGetInputSourceLocalizedName");
	//! @todo verify path

	return oxr_action_get_input_source_localized_name(
	    &log, sess, source, whichComponents, bufferCapacityInput,
	    bufferCountOutput, buffer);
}


/*
 *
 * Action set functions
 *
 */

XrResult
oxr_xrCreateActionSet(XrSession session,
                      const XrActionSetCreateInfo* createInfo,
                      XrActionSet* actionSet)
{
	struct oxr_action_set* act_set = NULL;
	struct oxr_session* sess = NULL;
	struct oxr_logger log;
	XrResult ret;
	OXR_VERIFY_SESSION_AND_INIT_LOG(&log, session, sess,
	                                "xrCreateActionSet");
	OXR_VERIFY_ARG_TYPE_AND_NULL(&log, createInfo,
	                             XR_TYPE_ACTION_SET_CREATE_INFO);
	OXR_VERIFY_ARG_NOT_NULL(&log, actionSet);
	OXR_VERIFY_ARG_SINGLE_LEVEL_FIXED_LENGTH_PATH(
	    &log, createInfo->actionSetName);
	OXR_VERIFY_ARG_LOCALIZED_NAME(&log, createInfo->localizedActionSetName);

	ret = oxr_action_set_create(&log, sess, createInfo, &act_set);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	*actionSet = oxr_action_set_to_openxr(act_set);

	return XR_SUCCESS;
}

XrResult
oxr_xrDestroyActionSet(XrActionSet actionSet)
{
	struct oxr_action_set* act_set;
	struct oxr_logger log;
	OXR_VERIFY_ACTIONSET_AND_INIT_LOG(&log, actionSet, act_set,
	                                  "xrDestroyActionSet");

	return oxr_handle_destroy(&log, &act_set->handle);
}


/*
 *
 * Action functions
 *
 */

XrResult
oxr_xrCreateAction(XrActionSet actionSet,
                   const XrActionCreateInfo* createInfo,
                   XrAction* action)
{
	struct oxr_action_set* act_set;
	struct oxr_action* act = NULL;
	struct oxr_logger log;
	XrResult ret;

	OXR_VERIFY_ACTIONSET_AND_INIT_LOG(&log, actionSet, act_set,
	                                  "xrCreateAction");
	OXR_VERIFY_ARG_TYPE_AND_NULL(&log, createInfo,
	                             XR_TYPE_ACTION_CREATE_INFO);
	OXR_VERIFY_ARG_SINGLE_LEVEL_FIXED_LENGTH_PATH(&log,
	                                              createInfo->actionName);
	OXR_VERIFY_ARG_LOCALIZED_NAME(&log, createInfo->localizedActionName);
	OXR_VERIFY_ARG_NOT_NULL(&log, action);

	struct oxr_instance* inst = act_set->sess->sys->inst;

	ret = oxr_verify_subaction_paths_create(
	    &log, inst, createInfo->countSubactionPaths,
	    createInfo->subactionPaths, "createInfo->subactionPaths");
	if (ret != XR_SUCCESS) {
		return ret;
	}

	ret = oxr_action_create(&log, act_set, createInfo, &act);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	*action = oxr_action_to_openxr(act);

	return XR_SUCCESS;
}

XrResult
oxr_xrDestroyAction(XrAction action)
{
	struct oxr_action* act;
	struct oxr_logger log;
	OXR_VERIFY_ACTION_AND_INIT_LOG(&log, action, act, "xrDestroyAction");

	return oxr_handle_destroy(&log, &act->handle);
}

XrResult
oxr_xrGetActionStateBoolean(XrAction action,
                            uint32_t countSubactionPaths,
                            const XrPath* subactionPaths,
                            XrActionStateBoolean* data)
{
	XrPath subactionPath = XR_NULL_PATH;
	struct oxr_sub_paths sub_paths = {0};
	struct oxr_action* act;
	struct oxr_logger log;
	XrResult ret;
	OXR_VERIFY_ACTION_AND_INIT_LOG(&log, action, act,
	                               "xrGetActionStateBoolean");
	OXR_VERIFY_ARG_TYPE_AND_NULL(&log, data, XR_TYPE_ACTION_STATE_BOOLEAN);
	OXR_VERIFY_SUBACTION_PATHS(&log, countSubactionPaths, subactionPaths);

	if (act->action_type != XR_INPUT_ACTION_TYPE_BOOLEAN) {
		return oxr_error(&log, XR_ERROR_ACTION_TYPE_MISMATCH,
		                 " not created with pose type");
	}

	// Trust me.
	if (countSubactionPaths > 1) {
		return oxr_error(&log, XR_ERROR_PATH_INVALID,
		                 " can not handle more then one subactionPath");
	}

	if (countSubactionPaths == 1) {
		subactionPath = subactionPaths[0];
	}

	ret = oxr_verify_subaction_path_get(&log, act->act_set->sess->sys->inst,
	                                    subactionPath, &act->sub_paths,
	                                    &sub_paths, "subactionPaths[0]");
	if (ret != XR_SUCCESS) {
		return ret;
	}

	return oxr_action_get_boolean(&log, act, sub_paths, data);
}

XrResult
oxr_xrGetActionStateVector1f(XrAction action,
                             uint32_t countSubactionPaths,
                             const XrPath* subactionPaths,
                             XrActionStateVector1f* data)
{
	XrPath subactionPath = XR_NULL_PATH;
	struct oxr_sub_paths sub_paths = {0};
	struct oxr_action* act;
	struct oxr_logger log;
	XrResult ret;
	OXR_VERIFY_ACTION_AND_INIT_LOG(&log, action, act,
	                               "xrGetActionStateVector1f");
	OXR_VERIFY_ARG_TYPE_AND_NULL(&log, data, XR_TYPE_ACTION_STATE_VECTOR1F);
	OXR_VERIFY_SUBACTION_PATHS(&log, countSubactionPaths, subactionPaths);

	if (act->action_type != XR_INPUT_ACTION_TYPE_VECTOR1F) {
		return oxr_error(&log, XR_ERROR_ACTION_TYPE_MISMATCH,
		                 " not created with float type");
	}

	// Trust me.
	if (countSubactionPaths > 1) {
		return oxr_error(&log, XR_ERROR_PATH_INVALID,
		                 " can not handle more then one subactionPath");
	}

	if (countSubactionPaths == 1) {
		subactionPath = subactionPaths[0];
	}

	ret = oxr_verify_subaction_path_get(&log, act->act_set->sess->sys->inst,
	                                    subactionPath, &act->sub_paths,
	                                    &sub_paths, "subactionPaths[0]");
	if (ret != XR_SUCCESS) {
		return ret;
	}

	return oxr_action_get_vector1f(&log, act, sub_paths, data);
}

XrResult
oxr_xrGetActionStateVector2f(XrAction action,
                             uint32_t countSubactionPaths,
                             const XrPath* subactionPaths,
                             XrActionStateVector2f* data)
{
	XrPath subactionPath = XR_NULL_PATH;
	struct oxr_sub_paths sub_paths = {0};
	struct oxr_action* act;
	struct oxr_logger log;
	XrResult ret;
	OXR_VERIFY_ACTION_AND_INIT_LOG(&log, action, act,
	                               "xrGetActionStateVector2f");
	OXR_VERIFY_ARG_TYPE_AND_NULL(&log, data, XR_TYPE_ACTION_STATE_VECTOR2F);
	OXR_VERIFY_SUBACTION_PATHS(&log, countSubactionPaths, subactionPaths);

	if (act->action_type != XR_INPUT_ACTION_TYPE_VECTOR2F) {
		return oxr_error(&log, XR_ERROR_ACTION_TYPE_MISMATCH,
		                 " not created with float[2] type");
	}

	// Trust me.
	if (countSubactionPaths > 1) {
		return oxr_error(&log, XR_ERROR_PATH_INVALID,
		                 " can not handle more then one subactionPath");
	}

	if (countSubactionPaths == 1) {
		subactionPath = subactionPaths[0];
	}

	ret = oxr_verify_subaction_path_get(&log, act->act_set->sess->sys->inst,
	                                    subactionPath, &act->sub_paths,
	                                    &sub_paths, "subactionPaths[0]");
	if (ret != XR_SUCCESS) {
		return ret;
	}

	return oxr_action_get_vector2f(&log, act, sub_paths, data);
}

XrResult
oxr_xrGetActionStatePose(XrAction action,
                         XrPath subactionPath,
                         XrActionStatePose* data)
{
	struct oxr_sub_paths sub_paths = {0};
	struct oxr_action* act;
	struct oxr_logger log;
	XrResult ret;
	OXR_VERIFY_ACTION_AND_INIT_LOG(&log, action, act,
	                               "xrGetActionStatePose");
	OXR_VERIFY_ARG_TYPE_AND_NULL(&log, data, XR_TYPE_ACTION_STATE_POSE);

	if (act->action_type != XR_INPUT_ACTION_TYPE_POSE) {
		return oxr_error(&log, XR_ERROR_ACTION_TYPE_MISMATCH,
		                 " not created with pose type");
	}

	ret = oxr_verify_subaction_path_get(&log, act->act_set->sess->sys->inst,
	                                    subactionPath, &act->sub_paths,
	                                    &sub_paths, "subactionPath");
	if (ret != XR_SUCCESS) {
		return ret;
	}

	return oxr_action_get_pose(&log, act, sub_paths, data);
}

XrResult
oxr_xrGetBoundSourcesForAction(XrAction action,
                               uint32_t sourceCapacityInput,
                               uint32_t* sourceCountOutput,
                               XrPath* sources)
{
	struct oxr_action* act;
	struct oxr_logger log;
	OXR_VERIFY_ACTION_AND_INIT_LOG(&log, action, act,
	                               "xrGetBoundSourcesForAction");

	return oxr_action_get_bound_sources(&log, act, sourceCapacityInput,
	                                    sourceCountOutput, sources);
}


/*
 *
 * Haptic feedback functions.
 *
 */

XrResult
oxr_xrApplyHapticFeedback(XrAction hapticAction,
                          uint32_t countSubactionPaths,
                          const XrPath* subactionPaths,
                          const XrHapticBaseHeader* hapticEvent)
{
	struct oxr_action* act;
	struct oxr_logger log;
	OXR_VERIFY_ACTION_AND_INIT_LOG(&log, hapticAction, act,
	                               "xrApplyHapticFeedback");
	OXR_VERIFY_SUBACTION_PATHS(&log, countSubactionPaths, subactionPaths);
	//! @todo verify paths

	if (act->action_type != XR_OUTPUT_ACTION_TYPE_VIBRATION) {
		return oxr_error(&log, XR_ERROR_ACTION_TYPE_MISMATCH,
		                 " not created with output vibration type");
	}

	return oxr_action_apply_haptic_feedback(&log, act, countSubactionPaths,
	                                        subactionPaths, hapticEvent);
}

XrResult
oxr_xrStopHapticFeedback(XrAction hapticAction,
                         uint32_t countSubactionPaths,
                         const XrPath* subactionPaths)
{
	struct oxr_action* act;
	struct oxr_logger log;
	OXR_VERIFY_ACTION_AND_INIT_LOG(&log, hapticAction, act,
	                               "xrStopHapticFeedback");
	OXR_VERIFY_SUBACTION_PATHS(&log, countSubactionPaths, subactionPaths);
	//! @todo verify paths

	if (act->action_type != XR_OUTPUT_ACTION_TYPE_VIBRATION) {
		return oxr_error(&log, XR_ERROR_ACTION_TYPE_MISMATCH,
		                 " not created with output vibration type");
	}

	return oxr_action_stop_haptic_feedback(&log, act, countSubactionPaths,
	                                       subactionPaths);
}
