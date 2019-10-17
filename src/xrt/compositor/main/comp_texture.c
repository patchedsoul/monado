// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Texture code for the main compositor.
 * @author Pete Black <pblack@collabora.com>
 * @ingroup comp
 */

#include <stdio.h>
#include <stdlib.h>

#include "util/u_misc.h"

#include "main/comp_compositor.h"

struct comp_texture *
comp_texture_create(struct comp_compositor *c,
                    int64_t format,
                    uint32_t sample_count,
                    uint32_t width,
                    uint32_t height,
                    uint32_t face_count,
                    uint32_t array_size,
                    uint32_t mip_count)
{
	struct comp_texture *ct = U_TYPED_CALLOC(struct comp_texture);
	VkMemoryRequirements vmr;
	VkMemoryAllocateInfo vmi;
	VkImageCreateInfo image_create_info = {
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = format,
	    .mipLevels = 1,
	    .arrayLayers = 1,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_LINEAR,
	    .usage = VK_IMAGE_USAGE_SAMPLED_BIT,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	    .initialLayout = VK_IMAGE_LAYOUT_PREINITIALIZED,
	    .extent = {.width = width, .height = height, 1}};
     VkResult ret = c->vk.vkCreateImage(c->vk.device, &image_create_info, NULL, &ct->image));
     if (ret != VK_SUCCESS) {
	     COMP_ERROR(c, "vkCreateImage: %s", vk_result_string(ret));
	     return NULL;
     }
     vkGetImageMemoryRequirements(c->vk.device, &ct->image, &vmr);
     vmi.allocationSize = vmr.size;
     vmi.memoryTypeIndex = c->vk.device_memory_props getMemoryType(
	 vmr.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
	                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
     VK_CHECK_RESULT(
	 vkAllocateMemory(device, &memAllocInfo, nullptr, &mappableMemory));
     VK_CHECK_RESULT(
	 vkBindImageMemory(device, mappableImage, mappableMemory, 0));

     // Map image memory
     void *data;
     VK_CHECK_RESULT(
	 vkMapMemory(device, mappableMemory, 0, memReqs.size, 0, &data));
     // Copy image data of the first mip level into memory
     memcpy(data, tex2D[0].data(), tex2D[0].size());
     vkUnmapMemory(device, mappableMemory);

     // Linear tiled images don't need to be staged and can be directly used as
     // textures
     texture.image = mappableImage;
     texture.deviceMemory = mappableMemory;
     texture.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

     // Setup image memory barrier transfer image to shader read layout
     VkCommandBuffer copyCmd = VulkanExampleBase::createCommandBuffer(
	 VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

     // The sub resource range describes the regions of the image we will be
     // transition
     VkImageSubresourceRange subresourceRange = {};
     subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
     subresourceRange.baseMipLevel = 0;
     subresourceRange.levelCount = 1;
     subresourceRange.layerCount = 1;

     // Transition the texture image layout to shader read, so it can be sampled
     // from
     VkImageMemoryBarrier imageMemoryBarrier =
	 vks::initializers::imageMemoryBarrier();
     ;
     imageMemoryBarrier.image = texture.image;
     imageMemoryBarrier.subresourceRange = subresourceRange;
     imageMemoryBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
     imageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
     imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_PREINITIALIZED;
     imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

     // Insert a memory dependency at the proper pipeline stages that will
     // execute the image layout transition Source pipeline stage is host
     // write/read exection (VK_PIPELINE_STAGE_HOST_BIT) Destination pipeline
     // stage fragment shader access (VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT)
     vkCmdPipelineBarrier(copyCmd, VK_PIPELINE_STAGE_HOST_BIT,
	                  VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr,
	                  0, nullptr, 1, &imageMemoryBarrier);

     VulkanExampleBase::flushCommandBuffer(copyCmd, queue, true);
}

// Create a texture sampler
// In Vulkan textures are accessed by samplers
// This separates all the sampling information from the texture data. This means
// you could have multiple sampler objects for the same texture with different
// settings Note: Similar to the samplers available with OpenGL 3.3
VkSamplerCreateInfo sampler = vks::initializers::samplerCreateInfo();
sampler.magFilter = VK_FILTER_LINEAR;
sampler.minFilter = VK_FILTER_LINEAR;
sampler.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
sampler.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
sampler.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
sampler.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
sampler.mipLodBias = 0.0f;
sampler.compareOp = VK_COMPARE_OP_NEVER;
sampler.minLod = 0.0f;
// Set max level-of-detail to mip level count of the texture
sampler.maxLod = (useStaging) ? (float)texture.mipLevels : 0.0f;
// Enable anisotropic filtering
// This feature is optional, so we must check if it's supported on the device
if (vulkanDevice->features.samplerAnisotropy) {
	// Use max. level of anisotropy for this example
	sampler.maxAnisotropy =
	    vulkanDevice->properties.limits.maxSamplerAnisotropy;
	sampler.anisotropyEnable = VK_TRUE;
} else {
	// The device does not support anisotropic filtering
	sampler.maxAnisotropy = 1.0;
	sampler.anisotropyEnable = VK_FALSE;
}
sampler.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
VK_CHECK_RESULT(vkCreateSampler(device, &sampler, nullptr, &texture.sampler));

// Create image view
// Textures are not directly accessed by the shaders and
// are abstracted by image views containing additional
// information and sub resource ranges
VkImageViewCreateInfo view = vks::initializers::imageViewCreateInfo();
view.viewType = VK_IMAGE_VIEW_TYPE_2D;
view.format = format;
view.components = {VK_COMPONENT_SWIZZLE_R, VK_COMPONENT_SWIZZLE_G,
                   VK_COMPONENT_SWIZZLE_B, VK_COMPONENT_SWIZZLE_A};
// The subresource range describes the set of mip levels (and array layers) that
// can be accessed through this image view It's possible to create multiple
// image views for a single image referring to different (and/or overlapping)
// ranges of the image
view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
view.subresourceRange.baseMipLevel = 0;
view.subresourceRange.baseArrayLayer = 0;
view.subresourceRange.layerCount = 1;
// Linear tiling usually won't support mip maps
// Only set mip map count if optimal tiling is used
view.subresourceRange.levelCount = (useStaging) ? texture.mipLevels : 1;
// The view will be based on the texture's image
view.image = texture.image;
VK_CHECK_RESULT(vkCreateImageView(device, &view, nullptr, &texture.view));
}

// Free all Vulkan resources used by a texture object
void
comp_texture_destroy(comp_texture texture);
{
	vkDestroyImageView(device, texture.view, nullptr);
	vkDestroyImage(device, texture.image, nullptr);
	vkDestroySampler(device, texture.sampler, nullptr);
	vkFreeMemory(device, texture.deviceMemory, nullptr);
}
}

static void
comp_texture_destroy(struct xrt_swapchain *xsc)
{}

static uint32_t
get_memory_type(uint32_t typeBits,
                VkMemoryPropertyFlags properties,
                VkBool32 *memTypeFound = nullptr)
{
	for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; i++) {
		if ((typeBits & 1) == 1) {
			if ((memoryProperties.memoryTypes[i].propertyFlags &
			     properties) == properties) {
				if (memTypeFound) {
					*memTypeFound = true;
				}
				return i;
			}
		}
		typeBits >>= 1;
	}

	if (memTypeFound) {
		*memTypeFound = false;
		return 0;
	} else {
		throw std::runtime_error(
		    "Could not find a matching memory type");
	}
}
