// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Common Vulkan code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @ingroup comp_common
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util/u_misc.h"
#include "util/u_debug.h"

#include "common/comp_vk.h"


/*
 *
 * String helper functions.
 *
 */

#define ENUM_TO_STR(r)                                                         \
	case r: return #r

const char *
vk_result_string(VkResult code)
{
	switch (code) {
		ENUM_TO_STR(VK_SUCCESS);
		ENUM_TO_STR(VK_NOT_READY);
		ENUM_TO_STR(VK_TIMEOUT);
		ENUM_TO_STR(VK_EVENT_SET);
		ENUM_TO_STR(VK_EVENT_RESET);
		ENUM_TO_STR(VK_INCOMPLETE);
		ENUM_TO_STR(VK_ERROR_OUT_OF_HOST_MEMORY);
		ENUM_TO_STR(VK_ERROR_OUT_OF_DEVICE_MEMORY);
		ENUM_TO_STR(VK_ERROR_INITIALIZATION_FAILED);
		ENUM_TO_STR(VK_ERROR_DEVICE_LOST);
		ENUM_TO_STR(VK_ERROR_MEMORY_MAP_FAILED);
		ENUM_TO_STR(VK_ERROR_LAYER_NOT_PRESENT);
		ENUM_TO_STR(VK_ERROR_EXTENSION_NOT_PRESENT);
		ENUM_TO_STR(VK_ERROR_FEATURE_NOT_PRESENT);
		ENUM_TO_STR(VK_ERROR_INCOMPATIBLE_DRIVER);
		ENUM_TO_STR(VK_ERROR_TOO_MANY_OBJECTS);
		ENUM_TO_STR(VK_ERROR_FORMAT_NOT_SUPPORTED);
		ENUM_TO_STR(VK_ERROR_SURFACE_LOST_KHR);
		ENUM_TO_STR(VK_ERROR_NATIVE_WINDOW_IN_USE_KHR);
		ENUM_TO_STR(VK_SUBOPTIMAL_KHR);
		ENUM_TO_STR(VK_ERROR_OUT_OF_DATE_KHR);
		ENUM_TO_STR(VK_ERROR_INCOMPATIBLE_DISPLAY_KHR);
		ENUM_TO_STR(VK_ERROR_VALIDATION_FAILED_EXT);
		ENUM_TO_STR(VK_ERROR_INVALID_SHADER_NV);
		ENUM_TO_STR(VK_ERROR_INVALID_EXTERNAL_HANDLE);
	default: return "UNKNOWN RESULT";
	}
}

const char *
vk_color_format_string(VkFormat code)
{
	switch (code) {
		ENUM_TO_STR(VK_FORMAT_B8G8R8A8_UNORM);
		ENUM_TO_STR(VK_FORMAT_UNDEFINED);
		ENUM_TO_STR(VK_FORMAT_R8G8B8A8_SRGB);
		ENUM_TO_STR(VK_FORMAT_B8G8R8A8_SRGB);
		ENUM_TO_STR(VK_FORMAT_R8G8B8_SRGB);
		ENUM_TO_STR(VK_FORMAT_B8G8R8_SRGB);
		ENUM_TO_STR(VK_FORMAT_R5G6B5_UNORM_PACK16);
		ENUM_TO_STR(VK_FORMAT_B5G6R5_UNORM_PACK16);
		ENUM_TO_STR(VK_FORMAT_D32_SFLOAT_S8_UINT);
		ENUM_TO_STR(VK_FORMAT_D32_SFLOAT);
		ENUM_TO_STR(VK_FORMAT_D24_UNORM_S8_UINT);
		ENUM_TO_STR(VK_FORMAT_D16_UNORM_S8_UINT);
		ENUM_TO_STR(VK_FORMAT_D16_UNORM);
	default: return "UNKNOWN FORMAT";
	}
}

const char *
vk_present_mode_string(VkPresentModeKHR code)
{
	switch (code) {
		ENUM_TO_STR(VK_PRESENT_MODE_FIFO_KHR);
		ENUM_TO_STR(VK_PRESENT_MODE_MAILBOX_KHR);
		ENUM_TO_STR(VK_PRESENT_MODE_IMMEDIATE_KHR);
		ENUM_TO_STR(VK_PRESENT_MODE_FIFO_RELAXED_KHR);
		ENUM_TO_STR(VK_PRESENT_MODE_SHARED_DEMAND_REFRESH_KHR);
		ENUM_TO_STR(VK_PRESENT_MODE_SHARED_CONTINUOUS_REFRESH_KHR);
	default: return "UNKNOWN MODE";
	}
}

const char *
vk_power_state_string(VkDisplayPowerStateEXT code)
{
	switch (code) {
		ENUM_TO_STR(VK_DISPLAY_POWER_STATE_OFF_EXT);
		ENUM_TO_STR(VK_DISPLAY_POWER_STATE_SUSPEND_EXT);
		ENUM_TO_STR(VK_DISPLAY_POWER_STATE_ON_EXT);
	default: return "UNKNOWN MODE";
	}
}

const char *
vk_color_space_string(VkColorSpaceKHR code)
{
	switch (code) {
		ENUM_TO_STR(VK_COLORSPACE_SRGB_NONLINEAR_KHR);
	default: return "UNKNOWN COLOR SPACE";
	}
}


/*
 *
 * Functions.
 *
 */

bool
vk_get_memory_type(struct vk_bundle *vk,
                   uint32_t type_bits,
                   VkMemoryPropertyFlags memory_props,
                   uint32_t *out_type_id)
{

	for (uint32_t i = 0; i < vk->device_memory_props.memoryTypeCount; i++) {
		uint32_t propertyFlags =
		    vk->device_memory_props.memoryTypes[i].propertyFlags;
		if ((type_bits & 1) == 1) {
			if ((propertyFlags & memory_props) == memory_props) {
				*out_type_id = i;
				return true;
			}
		}
		type_bits >>= 1;
	}

	VK_DEBUG(vk, "Could not find memory type!");

	return false;
}

VkResult
vk_alloc_and_bind_image_memory(struct vk_bundle *vk,
                               VkImage image,
                               size_t max_size,
                               const void *pNext_for_allocate,
                               VkDeviceMemory *out_mem,
                               VkDeviceSize *out_size)
{
	VkMemoryRequirements memory_requirements;
	vk->vkGetImageMemoryRequirements(vk->device, image,
	                                 &memory_requirements);
	if (memory_requirements.size > max_size) {
		VK_ERROR(vk,
		         "client_vk_swapchain - Got too little memory "
		         "%u vs %u\n",
		         (uint32_t)memory_requirements.size,
		         (uint32_t)max_size);
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}
	if (out_size != NULL) {
		*out_size = memory_requirements.size;
	}

	uint32_t memory_type_index = UINT32_MAX;
	if (!vk_get_memory_type(vk, memory_requirements.memoryTypeBits,
	                        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
	                        &memory_type_index)) {
		VK_ERROR(c, "vk_get_memory_type failed!");
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}

	VkMemoryAllocateInfo alloc_info = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
	    .pNext = pNext_for_allocate,
	    .allocationSize = memory_requirements.size,
	    .memoryTypeIndex = memory_type_index,
	};

	VkDeviceMemory device_memory = VK_NULL_HANDLE;
	VkResult ret =
	    vk->vkAllocateMemory(vk->device, &alloc_info, NULL, &device_memory);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkAllocateMemory: %s", vk_result_string(ret));
		return ret;
	}

	// Bind the memory to the image.
	ret = vk->vkBindImageMemory(vk->device, image, device_memory, 0);
	if (ret != VK_SUCCESS) {
		// Clean up memory
		vk->vkFreeMemory(vk->device, device_memory, NULL);
		VK_ERROR(vk, "vkBindImageMemory: %s", vk_result_string(ret));
		return ret;
	}

	*out_mem = device_memory;
	return ret;
}

VkResult
vk_create_image_simple(struct vk_bundle *vk,
                       uint32_t width,
                       uint32_t height,
                       VkFormat format,
                       VkDeviceMemory *out_mem,
                       VkImage *out_image)
{
	VkImageUsageFlags usage_flags = 0;
	usage_flags |= VK_IMAGE_USAGE_SAMPLED_BIT;
	usage_flags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;

	VkImageCreateInfo image_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
	    .pNext = NULL,
	    .flags = 0,
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = format,
	    .extent =
	        {
	            .width = width,
	            .height = height,
	            .depth = 1,
	        },
	    .mipLevels = 1,
	    .arrayLayers = 1,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_LINEAR,
	    .usage = usage_flags,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	    .queueFamilyIndexCount = 0,
	    .pQueueFamilyIndices = NULL,
	    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};

	VkImage image;
	VkResult ret = vk->vkCreateImage(vk->device, &image_info, NULL, &image);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreateImage: %s", vk_result_string(ret));
		// Nothing to cleanup
		return ret;
	}

	ret = vk_alloc_and_bind_image_memory(vk, image, SIZE_MAX, NULL, out_mem,
	                                     NULL);
	if (ret != VK_SUCCESS) {
		// Clean up image
		vk->vkDestroyImage(vk->device, image, NULL);
		return ret;
	}

	*out_image = image;
	return ret;
}

VkResult
vk_create_image_from_fd(struct vk_bundle *vk,
                        enum xrt_swapchain_usage_bits swapchain_usage,
                        int64_t format,
                        uint32_t width,
                        uint32_t height,
                        uint32_t array_size,
                        uint32_t mip_count,
                        struct xrt_image_fd *image_fd,
                        VkImage *out_image,
                        VkDeviceMemory *out_mem)
{
	VkImageUsageFlags image_usage = (VkImageUsageFlags)0;
	VkImage image = VK_NULL_HANDLE;
	VkResult ret = VK_SUCCESS;

	VkExternalMemoryImageCreateInfoKHR external_memory_image_create_info = {
	    .sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO_KHR,
	    .pNext = NULL,
	    .handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR,
	};

	if ((swapchain_usage & XRT_SWAPCHAIN_USAGE_COLOR) != 0) {
		image_usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
	}
	if ((swapchain_usage & XRT_SWAPCHAIN_USAGE_DEPTH_STENCIL) != 0) {
		image_usage |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
	}
	if ((swapchain_usage & XRT_SWAPCHAIN_USAGE_UNORDERED_ACCESS) != 0) {
		image_usage |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
	}
	if ((swapchain_usage & XRT_SWAPCHAIN_USAGE_TRANSFER_SRC) != 0) {
		image_usage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
	}
	if ((swapchain_usage & XRT_SWAPCHAIN_USAGE_TRANSFER_DST) != 0) {
		image_usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	}
	if ((swapchain_usage & XRT_SWAPCHAIN_USAGE_SAMPLED) != 0) {
		image_usage |= VK_IMAGE_USAGE_SAMPLED_BIT;
	}

	VkImageCreateInfo info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
	    .pNext = &external_memory_image_create_info,
	    .flags = 0,
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = (VkFormat)format,
	    .extent = {.width = width, .height = height, .depth = 1},
	    .mipLevels = mip_count,
	    .arrayLayers = array_size,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_OPTIMAL,
	    .usage = image_usage,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	    .queueFamilyIndexCount = 0,
	    .pQueueFamilyIndices = NULL,
	    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};

	ret = vk->vkCreateImage(vk->device, &info, NULL, &image);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreateImage: %s", vk_result_string(ret));
		// Nothing to cleanup
		return ret;
	}

	VkImportMemoryFdInfoKHR import_memory_info = {
	    .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR,
	    .pNext = NULL,
	    .handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR,
	    .fd = image_fd->fd,
	};
	VkMemoryDedicatedAllocateInfoKHR dedicated_memory_info = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO_KHR,
	    .pNext = &import_memory_info,
	    .image = image,
	    .buffer = VK_NULL_HANDLE,
	};
	ret = vk_alloc_and_bind_image_memory(
	    vk, image, image_fd->size, &dedicated_memory_info, out_mem, NULL);
	if (ret != VK_SUCCESS) {
		vk->vkDestroyImage(vk->device, image, NULL);
		return ret;
	}

	*out_image = image;
	return ret;
}

VkResult
vk_create_sampler(struct vk_bundle *vk, VkSampler *out_sampler)
{
	VkSampler sampler;
	VkResult ret;

	VkSamplerCreateInfo info = {
	    .sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO,
	    .pNext = NULL,
	    .flags = 0,
	    .magFilter = VK_FILTER_LINEAR,
	    .minFilter = VK_FILTER_LINEAR,
	    .mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR,
	    .addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER,
	    .addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER,
	    .addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER,
	    .mipLodBias = 0.0f,
	    .anisotropyEnable = VK_FALSE,
	    .maxAnisotropy = 1.0f,
	    .compareEnable = VK_FALSE,
	    .compareOp = VK_COMPARE_OP_NEVER,
	    .minLod = 0.0f,
	    .maxLod = 1.0f,
	    .borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK,
	    .unnormalizedCoordinates = VK_FALSE,
	};

	ret = vk->vkCreateSampler(vk->device, &info, NULL, &sampler);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreateSampler: %s", vk_result_string(ret));
		return ret;
	}

	*out_sampler = sampler;

	return VK_SUCCESS;
}

VkResult
vk_create_view(struct vk_bundle *vk,
               VkImage image,
               VkFormat format,
               VkImageSubresourceRange subresource_range,
               VkImageView *out_view)
{
	VkImageView view;
	VkResult ret;

	VkImageViewCreateInfo imageView = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
	    .pNext = NULL,
	    .flags = 0,
	    .image = image,
	    .viewType = VK_IMAGE_VIEW_TYPE_2D,
	    .format = format,
	    .components =
	        {
	            .r = VK_COMPONENT_SWIZZLE_R,
	            .g = VK_COMPONENT_SWIZZLE_G,
	            .b = VK_COMPONENT_SWIZZLE_B,
	            .a = VK_COMPONENT_SWIZZLE_A,
	        },
	    .subresourceRange = subresource_range,
	};

	ret = vk->vkCreateImageView(vk->device, &imageView, NULL, &view);
	if (ret != VK_SUCCESS) {
		VK_ERROR(c, "vkCreateImageView: %s", vk_result_string(ret));
		return ret;
	}

	*out_view = view;

	return VK_SUCCESS;
}


/*
 *
 * Command buffer code.
 *
 */

VkResult
vk_init_cmd_buffer(struct vk_bundle *vk, VkCommandBuffer *out_cmd_buffer)
{
	VkCommandBuffer cmd_buffer;
	VkResult ret;

	// Allocate the command buffer.
	VkCommandBufferAllocateInfo cmd_buffer_info = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
	    .pNext = NULL,
	    .commandPool = vk->cmd_pool,
	    .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
	    .commandBufferCount = 1,
	};

	ret = vk->vkAllocateCommandBuffers(vk->device, &cmd_buffer_info,
	                                   &cmd_buffer);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkAllocateCommandBuffers: %s",
		         vk_result_string(ret));
		// Nothing to cleanup
		return ret;
	}

	// Start the command buffer as well.
	VkCommandBufferBeginInfo begin_info = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
	    .pNext = NULL,
	    .flags = 0,
	    .pInheritanceInfo = NULL,
	};
	ret = vk->vkBeginCommandBuffer(cmd_buffer, &begin_info);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkBeginCommandBuffer: %s", vk_result_string(ret));
		goto err_buffer;
	}

	*out_cmd_buffer = cmd_buffer;

	return VK_SUCCESS;


err_buffer:
	vk->vkFreeCommandBuffers(vk->device, vk->cmd_pool, 1, &cmd_buffer);

	return ret;
}

VkResult
vk_set_image_layout(struct vk_bundle *vk,
                    VkCommandBuffer cmd_buffer,
                    VkImage image,
                    VkAccessFlags src_access_mask,
                    VkAccessFlags dst_access_mask,
                    VkImageLayout old_layout,
                    VkImageLayout new_layout,
                    VkImageSubresourceRange subresource_range)
{
	VkImageMemoryBarrier barrier = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .pNext = 0,
	    .srcAccessMask = src_access_mask,
	    .dstAccessMask = dst_access_mask,
	    .oldLayout = old_layout,
	    .newLayout = new_layout,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = image,
	    .subresourceRange = subresource_range,
	};

	vk->vkCmdPipelineBarrier(cmd_buffer, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
	                         VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, NULL,
	                         0, NULL, 1, &barrier);

	return VK_SUCCESS;
}

VkResult
vk_submit_cmd_buffer(struct vk_bundle *vk, VkCommandBuffer cmd_buffer)
{
	VkResult ret = VK_SUCCESS;
	VkQueue queue;
	VkFence fence;
	VkFenceCreateInfo fence_info = {
	    .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
	    .pNext = NULL,
	    .flags = 0,
	};
	VkSubmitInfo submitInfo = {
	    .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
	    .pNext = NULL,
	    .waitSemaphoreCount = 0,
	    .pWaitSemaphores = NULL,
	    .pWaitDstStageMask = NULL,
	    .commandBufferCount = 1,
	    .pCommandBuffers = &cmd_buffer,
	    .signalSemaphoreCount = 0,
	    .pSignalSemaphores = NULL,
	};

	// Finish the command buffer first.
	ret = vk->vkEndCommandBuffer(cmd_buffer);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkEndCommandBuffer: %s", vk_result_string(ret));
		goto out;
	}

	// Get the queue.
	vk->vkGetDeviceQueue(vk->device, vk->queue_family_index, 0, &queue);

	// Create the fence.
	ret = vk->vkCreateFence(vk->device, &fence_info, NULL, &fence);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreateFence: %s", vk_result_string(ret));
		goto out;
	}

	// Do the actual submitting.

	ret = vk->vkQueueSubmit(queue, 1, &submitInfo, fence);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "Error: Could not submit queue.\n");
		goto out_fence;
	}

	// Then wait for the fence.
	ret = vk->vkWaitForFences(vk->device, 1, &fence, VK_TRUE, 1000000000);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkWaitForFences: %s", vk_result_string(ret));
		goto out_fence;
	}

	// Yes fall through.

out_fence:
	vk->vkDestroyFence(vk->device, fence, NULL);
out:
	vk->vkFreeCommandBuffers(vk->device, vk->cmd_pool, 1, &cmd_buffer);

	return ret;
}

VkResult
vk_init_cmd_pool(struct vk_bundle *vk)
{
	VkCommandPoolCreateInfo cmd_pool_info = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
	    .pNext = NULL,
	    .flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
	    .queueFamilyIndex = vk->queue_family_index,
	};

	VkResult ret;
	ret = vk->vkCreateCommandPool(vk->device, &cmd_pool_info, NULL,
	                              &vk->cmd_pool);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreateCommandPool: %s", vk_result_string(ret));
	}

	return ret;
}


/*
 *
 * Debug code.
 *
 */

#define ENUM_TO_STR(r)                                                         \
	case r: return #r

static const char *
vk_debug_report_string(VkDebugReportFlagsEXT code)
{
	switch (code) {
		ENUM_TO_STR(VK_DEBUG_REPORT_INFORMATION_BIT_EXT);
		ENUM_TO_STR(VK_DEBUG_REPORT_WARNING_BIT_EXT);
		ENUM_TO_STR(VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT);
		ENUM_TO_STR(VK_DEBUG_REPORT_ERROR_BIT_EXT);
		ENUM_TO_STR(VK_DEBUG_REPORT_DEBUG_BIT_EXT);
		ENUM_TO_STR(VK_DEBUG_REPORT_FLAG_BITS_MAX_ENUM_EXT);
	}
	return "UNKNOWN REPORT";
}

static VkBool32 VKAPI_PTR
_validation_cb(VkDebugReportFlagsEXT flags,
               VkDebugReportObjectTypeEXT object_type,
               uint64_t object,
               size_t location,
               int32_t message_code,
               const char *layer_prefix,
               const char *message,
               void *user_data)
{
	fprintf(stderr, "%s %s %lu:%d: %s\n", vk_debug_report_string(flags),
	        layer_prefix, location, message_code, message);
	return VK_FALSE;
}

DEBUG_GET_ONCE_BOOL_OPTION(vulkan_spew, "XRT_COMPOSITOR_VULKAN_SPEW", false)

void
vk_init_validation_callback(struct vk_bundle *vk)
{
	VkDebugReportFlagsEXT flags = 0;
	flags |= VK_DEBUG_REPORT_ERROR_BIT_EXT;
	flags |= VK_DEBUG_REPORT_WARNING_BIT_EXT;

	if (debug_get_bool_option_vulkan_spew()) {
		flags |= VK_DEBUG_REPORT_INFORMATION_BIT_EXT;
		flags |= VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT;
		flags |= VK_DEBUG_REPORT_DEBUG_BIT_EXT;
	}

	VkDebugReportCallbackCreateInfoEXT info = {
	    .sType = VK_STRUCTURE_TYPE_DEBUG_REPORT_CREATE_INFO_EXT,
	    .pNext = NULL,
	    .flags = flags,
	    .pfnCallback = _validation_cb,
	    .pUserData = NULL,
	};

	vk->vkCreateDebugReportCallbackEXT(vk->instance, &info, NULL,
	                                   &vk->debug_report_cb);
}

void
vk_destroy_validation_callback(struct vk_bundle *vk)
{
	if (vk->debug_report_cb != VK_NULL_HANDLE) {
		vk->vkDestroyDebugReportCallbackEXT(vk->instance,
		                                    vk->debug_report_cb, NULL);
		vk->debug_report_cb = VK_NULL_HANDLE;
	}
}

/*
 *
 * Function getting code.
 *
 */

#define GET_PROC(vk, name) (PFN_##name) vk->vkGetInstanceProcAddr(NULL, #name);

#define GET_INS_PROC(vk, name)                                                 \
	(PFN_##name) vk->vkGetInstanceProcAddr(vk->instance, #name);

#define GET_DEV_PROC(vk, name)                                                 \
	(PFN_##name) vk->vkGetDeviceProcAddr(vk->device, #name);

VkResult
vk_get_loader_functions(struct vk_bundle *vk, PFN_vkGetInstanceProcAddr g)
{
	vk->vkGetInstanceProcAddr = g;

	// Fill in all loader functions.
	// clang-format off
	vk->vkCreateInstance = GET_PROC(vk, vkCreateInstance);
	// clang-format on

	return VK_SUCCESS;
}

VkResult
vk_get_instance_functions(struct vk_bundle *vk)
{
	// clang-format off
	vk->vkDestroyInstance                         = GET_INS_PROC(vk, vkDestroyInstance);
	vk->vkGetDeviceProcAddr                       = GET_INS_PROC(vk, vkGetDeviceProcAddr);
	vk->vkCreateDevice                            = GET_INS_PROC(vk, vkCreateDevice);
	vk->vkEnumeratePhysicalDevices                = GET_INS_PROC(vk, vkEnumeratePhysicalDevices);
	vk->vkGetPhysicalDeviceProperties             = GET_INS_PROC(vk, vkGetPhysicalDeviceProperties);
	vk->vkGetPhysicalDeviceMemoryProperties       = GET_INS_PROC(vk, vkGetPhysicalDeviceMemoryProperties);
	vk->vkGetPhysicalDeviceQueueFamilyProperties  = GET_INS_PROC(vk, vkGetPhysicalDeviceQueueFamilyProperties);
	vk->vkCreateDebugReportCallbackEXT            = GET_INS_PROC(vk, vkCreateDebugReportCallbackEXT);
	vk->vkDestroyDebugReportCallbackEXT           = GET_INS_PROC(vk, vkDestroyDebugReportCallbackEXT);
	vk->vkDestroySurfaceKHR                       = GET_INS_PROC(vk, vkDestroySurfaceKHR);
	vk->vkGetPhysicalDeviceSurfaceCapabilitiesKHR = GET_INS_PROC(vk, vkGetPhysicalDeviceSurfaceCapabilitiesKHR);
	vk->vkGetPhysicalDeviceSurfaceFormatsKHR      = GET_INS_PROC(vk, vkGetPhysicalDeviceSurfaceFormatsKHR);
	vk->vkGetPhysicalDeviceSurfacePresentModesKHR = GET_INS_PROC(vk, vkGetPhysicalDeviceSurfacePresentModesKHR);
	vk->vkGetPhysicalDeviceSurfaceSupportKHR      = GET_INS_PROC(vk, vkGetPhysicalDeviceSurfaceSupportKHR);

#ifdef VK_USE_PLATFORM_XCB_KHR
	vk->vkCreateXcbSurfaceKHR = GET_INS_PROC(vk, vkCreateXcbSurfaceKHR);
#endif

#ifdef VK_USE_PLATFORM_WAYLAND_KHR
	vk->vkCreateWaylandSurfaceKHR = GET_INS_PROC(vk, vkCreateWaylandSurfaceKHR);
#endif

#ifdef VK_USE_PLATFORM_XLIB_XRANDR_EXT
	vk->vkCreateDisplayPlaneSurfaceKHR               = GET_INS_PROC(vk, vkCreateDisplayPlaneSurfaceKHR);
	vk->vkGetDisplayPlaneCapabilitiesKHR             = GET_INS_PROC(vk, vkGetDisplayPlaneCapabilitiesKHR);
	vk->vkGetPhysicalDeviceDisplayPropertiesKHR      = GET_INS_PROC(vk, vkGetPhysicalDeviceDisplayPropertiesKHR);
	vk->vkGetPhysicalDeviceDisplayPlanePropertiesKHR = GET_INS_PROC(vk, vkGetPhysicalDeviceDisplayPlanePropertiesKHR);
	vk->vkGetDisplayModePropertiesKHR                = GET_INS_PROC(vk, vkGetDisplayModePropertiesKHR);
	vk->vkAcquireXlibDisplayEXT                      = GET_INS_PROC(vk, vkAcquireXlibDisplayEXT);
	vk->vkReleaseDisplayEXT                          = GET_INS_PROC(vk, vkReleaseDisplayEXT);
	vk->vkGetRandROutputDisplayEXT                   = GET_INS_PROC(vk, vkGetRandROutputDisplayEXT);
#endif
	// clang-format on

	return VK_SUCCESS;
}

static VkResult
vk_get_device_functions(struct vk_bundle *vk)
{
	// clang-format off
	vk->vkDestroyDevice               = GET_DEV_PROC(vk, vkDestroyDevice);
	vk->vkDeviceWaitIdle              = GET_DEV_PROC(vk, vkDeviceWaitIdle);
	vk->vkAllocateMemory              = GET_DEV_PROC(vk, vkAllocateMemory);
	vk->vkFreeMemory                  = GET_DEV_PROC(vk, vkFreeMemory);
	vk->vkMapMemory                   = GET_DEV_PROC(vk, vkMapMemory);
	vk->vkUnmapMemory                 = GET_DEV_PROC(vk, vkUnmapMemory);
	vk->vkGetMemoryFdKHR              = GET_DEV_PROC(vk, vkGetMemoryFdKHR);
	vk->vkCreateBuffer                = GET_DEV_PROC(vk, vkCreateBuffer);
	vk->vkDestroyBuffer               = GET_DEV_PROC(vk, vkDestroyBuffer);
	vk->vkBindBufferMemory            = GET_DEV_PROC(vk, vkBindBufferMemory);
	vk->vkGetBufferMemoryRequirements = GET_DEV_PROC(vk, vkGetBufferMemoryRequirements);
	vk->vkCreateImage                 = GET_DEV_PROC(vk, vkCreateImage);
	vk->vkGetImageMemoryRequirements  = GET_DEV_PROC(vk, vkGetImageMemoryRequirements);
    vk->vkBindImageMemory             = GET_DEV_PROC(vk, vkBindImageMemory);
	vk->vkDestroyImage                = GET_DEV_PROC(vk, vkDestroyImage);
	vk->vkCreateImageView             = GET_DEV_PROC(vk, vkCreateImageView);
	vk->vkDestroyImageView            = GET_DEV_PROC(vk, vkDestroyImageView);
	vk->vkCreateSampler               = GET_DEV_PROC(vk, vkCreateSampler);
	vk->vkDestroySampler              = GET_DEV_PROC(vk, vkDestroySampler);
	vk->vkCreateShaderModule          = GET_DEV_PROC(vk, vkCreateShaderModule);
	vk->vkDestroyShaderModule         = GET_DEV_PROC(vk, vkDestroyShaderModule);
	vk->vkCreateCommandPool           = GET_DEV_PROC(vk, vkCreateCommandPool);
	vk->vkDestroyCommandPool          = GET_DEV_PROC(vk, vkDestroyCommandPool);
	vk->vkAllocateCommandBuffers      = GET_DEV_PROC(vk, vkAllocateCommandBuffers);
	vk->vkBeginCommandBuffer          = GET_DEV_PROC(vk, vkBeginCommandBuffer);
	vk->vkCmdPipelineBarrier          = GET_DEV_PROC(vk, vkCmdPipelineBarrier);
	vk->vkCmdBeginRenderPass          = GET_DEV_PROC(vk, vkCmdBeginRenderPass);
	vk->vkCmdSetScissor               = GET_DEV_PROC(vk, vkCmdSetScissor);
	vk->vkCmdSetViewport              = GET_DEV_PROC(vk, vkCmdSetViewport);
	vk->vkCmdClearColorImage          = GET_DEV_PROC(vk, vkCmdClearColorImage);
	vk->vkCmdEndRenderPass            = GET_DEV_PROC(vk, vkCmdEndRenderPass);
	vk->vkCmdBindDescriptorSets       = GET_DEV_PROC(vk, vkCmdBindDescriptorSets);
	vk->vkCmdBindPipeline             = GET_DEV_PROC(vk, vkCmdBindPipeline);
	vk->vkCmdBindVertexBuffers        = GET_DEV_PROC(vk, vkCmdBindVertexBuffers);
	vk->vkCmdDraw                     = GET_DEV_PROC(vk, vkCmdDraw);
	vk->vkEndCommandBuffer            = GET_DEV_PROC(vk, vkEndCommandBuffer);
	vk->vkFreeCommandBuffers          = GET_DEV_PROC(vk, vkFreeCommandBuffers);
	vk->vkCreateRenderPass            = GET_DEV_PROC(vk, vkCreateRenderPass);
	vk->vkDestroyRenderPass           = GET_DEV_PROC(vk, vkDestroyRenderPass);
	vk->vkCreateFramebuffer           = GET_DEV_PROC(vk, vkCreateFramebuffer);
	vk->vkDestroyFramebuffer          = GET_DEV_PROC(vk, vkDestroyFramebuffer);
	vk->vkCreatePipelineCache         = GET_DEV_PROC(vk, vkCreatePipelineCache);
	vk->vkDestroyPipelineCache        = GET_DEV_PROC(vk, vkDestroyPipelineCache);
	vk->vkCreateDescriptorPool        = GET_DEV_PROC(vk, vkCreateDescriptorPool);
	vk->vkDestroyDescriptorPool       = GET_DEV_PROC(vk, vkDestroyDescriptorPool);
	vk->vkAllocateDescriptorSets      = GET_DEV_PROC(vk, vkAllocateDescriptorSets);
	vk->vkCreateGraphicsPipelines     = GET_DEV_PROC(vk, vkCreateGraphicsPipelines);
	vk->vkDestroyPipeline             = GET_DEV_PROC(vk, vkDestroyPipeline);
	vk->vkCreatePipelineLayout        = GET_DEV_PROC(vk, vkCreatePipelineLayout);
	vk->vkDestroyPipelineLayout       = GET_DEV_PROC(vk, vkDestroyPipelineLayout);
	vk->vkCreateDescriptorSetLayout   = GET_DEV_PROC(vk, vkCreateDescriptorSetLayout);
	vk->vkUpdateDescriptorSets        = GET_DEV_PROC(vk, vkUpdateDescriptorSets);
	vk->vkDestroyDescriptorSetLayout  = GET_DEV_PROC(vk, vkDestroyDescriptorSetLayout);
	vk->vkGetDeviceQueue              = GET_DEV_PROC(vk, vkGetDeviceQueue);
	vk->vkQueueSubmit                 = GET_DEV_PROC(vk, vkQueueSubmit);
	vk->vkQueueWaitIdle               = GET_DEV_PROC(vk, vkQueueWaitIdle);
	vk->vkCreateSemaphore             = GET_DEV_PROC(vk, vkCreateSemaphore);
	vk->vkDestroySemaphore            = GET_DEV_PROC(vk, vkDestroySemaphore);
	vk->vkCreateFence                 = GET_DEV_PROC(vk, vkCreateFence);
	vk->vkWaitForFences               = GET_DEV_PROC(vk, vkWaitForFences);
	vk->vkDestroyFence                = GET_DEV_PROC(vk, vkDestroyFence);
	vk->vkCreateSwapchainKHR          = GET_DEV_PROC(vk, vkCreateSwapchainKHR);
	vk->vkDestroySwapchainKHR         = GET_DEV_PROC(vk, vkDestroySwapchainKHR);
	vk->vkGetSwapchainImagesKHR       = GET_DEV_PROC(vk, vkGetSwapchainImagesKHR);
	vk->vkAcquireNextImageKHR         = GET_DEV_PROC(vk, vkAcquireNextImageKHR);
	vk->vkQueuePresentKHR             = GET_DEV_PROC(vk, vkQueuePresentKHR);
	// clang-format on

	return VK_SUCCESS;
}


/*
 *
 * Creation code.
 *
 */

static VkResult
vk_select_physical_device(struct vk_bundle *vk)
{
	VkPhysicalDevice physical_devices[16];
	uint32_t gpu_count = ARRAY_SIZE(physical_devices);
	VkResult ret;

	ret = vk->vkEnumeratePhysicalDevices(vk->instance, &gpu_count,
	                                     physical_devices);
	if (ret != VK_SUCCESS) {
		VK_DEBUG(vk, "vkEnumeratePhysicalDevices: %s",
		         vk_result_string(ret));
		return ret;
	}

	if (gpu_count < 1) {
		VK_DEBUG(vk, "No physical device found!");
		return VK_ERROR_DEVICE_LOST;
	}

	if (gpu_count > 1) {
		VK_DEBUG(vk, "Can not deal well with multiple devices.");
	}

	// as a first-step to 'intelligent' selection, prefer a 'discrete' gpu
	// if it is present
	uint32_t gpu_index = 0;
	for (uint32_t i = 0; i < gpu_count; i++) {
		VkPhysicalDeviceProperties pdp;
		vk->vkGetPhysicalDeviceProperties(physical_devices[i], &pdp);
		if (pdp.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
			gpu_index = i;
		}
	}

	vk->physical_device = physical_devices[gpu_index];

	// Debug print info.
	VkPhysicalDeviceProperties pdp;
	vk->vkGetPhysicalDeviceProperties(physical_devices[gpu_index], &pdp);
	VK_DEBUG(vk,
	         "Selected device:\n"
	         "\tname: %s\n"
	         "\tvendor: 0x%04x\n"
	         "\tproduct: 0x%04x\n"
	         "\tapiVersion: %u.%u.%u\n"
	         "\tdriverVersion: %u.%u.%u",
	         pdp.deviceName, pdp.vendorID, pdp.deviceID,
	         VK_VERSION_MAJOR(pdp.apiVersion),
	         VK_VERSION_MINOR(pdp.apiVersion),
	         VK_VERSION_PATCH(pdp.apiVersion),
	         VK_VERSION_MAJOR(pdp.driverVersion),
	         VK_VERSION_MINOR(pdp.driverVersion),
	         VK_VERSION_PATCH(pdp.driverVersion));

	// Fill out the device memory props as well.
	vk->vkGetPhysicalDeviceMemoryProperties(vk->physical_device,
	                                        &vk->device_memory_props);

	return VK_SUCCESS;
}

static VkResult
vk_find_graphics_queue(struct vk_bundle *vk, uint32_t *out_graphics_queue)
{
	/* Find the first graphics queue */
	uint32_t num_queues = 0;
	uint32_t i = 0;
	vk->vkGetPhysicalDeviceQueueFamilyProperties(vk->physical_device,
	                                             &num_queues, NULL);

	VkQueueFamilyProperties *queue_family_props =
	    U_TYPED_ARRAY_CALLOC(VkQueueFamilyProperties, num_queues);

	vk->vkGetPhysicalDeviceQueueFamilyProperties(
	    vk->physical_device, &num_queues, queue_family_props);

	if (num_queues == 0) {
		VK_DEBUG(vk, "Failed to get queue properties");
		goto err_free;
	}

	for (i = 0; i < num_queues; i++) {
		if (queue_family_props[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
			break;
		}
	}

	if (i >= num_queues) {
		VK_DEBUG(vk, "No graphics queue found");
		goto err_free;
	}

	*out_graphics_queue = i;

	free(queue_family_props);

	return VK_SUCCESS;

err_free:
	free(queue_family_props);
	return VK_ERROR_INITIALIZATION_FAILED;
}

VkResult
vk_create_device(struct vk_bundle *vk)
{
	VkResult ret;

	ret = vk_select_physical_device(vk);
	if (ret != VK_SUCCESS) {
		return ret;
	}

	VkPhysicalDeviceFeatures *enabled_features = NULL;

	float queue_priority = 0.0f;
	VkDeviceQueueCreateInfo queue_create_info = {
	    .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
	    .pNext = NULL,
	    .flags = 0,
	    .queueFamilyIndex = 0, // assigned valid value later
	    .queueCount = 1,
	    .pQueuePriorities = &queue_priority,
	};

	//! @todo why not vk->queue_family_index ?
	ret = vk_find_graphics_queue(vk, &queue_create_info.queueFamilyIndex);
	if (ret != VK_SUCCESS) {
		return ret;
	}

	const char *device_extensions[] = {
	    VK_KHR_SWAPCHAIN_EXTENSION_NAME,
	    VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME,
	    VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_FENCE_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_FENCE_FD_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_MEMORY_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_SEMAPHORE_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_SEMAPHORE_FD_EXTENSION_NAME,
	    // VK_EXT_DEBUG_MARKER_EXTENSION_NAME,
	};

	VkDeviceCreateInfo device_create_info = {
	    .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
	    .pNext = NULL,
	    .flags = 0,
	    .queueCreateInfoCount = 1,
	    .pQueueCreateInfos = &queue_create_info,
	    .enabledLayerCount = 0,
	    .ppEnabledLayerNames = NULL,
	    .enabledExtensionCount = ARRAY_SIZE(device_extensions),
	    .ppEnabledExtensionNames = device_extensions,
	    .pEnabledFeatures = enabled_features,
	};

	ret = vk->vkCreateDevice(vk->physical_device, &device_create_info, NULL,
	                         &vk->device);
	if (ret != VK_SUCCESS) {
		VK_DEBUG(vk, "vkCreateDevice: %s", vk_result_string(ret));
		return ret;
	}

	ret = vk_get_device_functions(vk);
	if (ret != VK_SUCCESS) {
		goto err_destroy;
	}

	return ret;

err_destroy:
	vk->vkDestroyDevice(vk->device, NULL);
	vk->device = NULL;

	return ret;
}

VkResult
vk_init_from_given(struct vk_bundle *vk,
                   PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr,
                   VkInstance instance,
                   VkPhysicalDevice physical_device,
                   VkDevice device,
                   uint32_t queue_family_index,
                   uint32_t queue_index)
{
	VkResult ret;

	// First memset it clear.
	U_ZERO(vk);

	vk->vkGetInstanceProcAddr = vkGetInstanceProcAddr;
	vk->instance = instance;
	vk->physical_device = physical_device;
	vk->device = device;
	vk->queue_family_index = queue_family_index;
	vk->queue_index = queue_index;

	// Not really needed but just in case.
	vk->vkCreateInstance = GET_PROC(vk, vkCreateInstance);

	// Fill in all instance functions.
	ret = vk_get_instance_functions(vk);
	if (ret != VK_SUCCESS) {
		goto err_memset;
	}

	// Fill out the device memory props here, as we are
	// passed a vulkan context and do not call selectPhysicalDevice()
	vk->vkGetPhysicalDeviceMemoryProperties(vk->physical_device,
	                                        &vk->device_memory_props);

	// Fill in all device functions.
	ret = vk_get_device_functions(vk);
	if (ret != VK_SUCCESS) {
		goto err_memset;
	}

	// Create the pool.
	ret = vk_init_cmd_pool(vk);
	if (ret != VK_SUCCESS) {
		goto err_memset;
	}

	return VK_SUCCESS;

err_memset:
	U_ZERO(vk);
	return ret;
}
